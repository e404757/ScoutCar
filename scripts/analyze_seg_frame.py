#!/usr/bin/env python3
"""对一帧前视画面复现 YOLOv5-seg 后处理，定位 road/barrier 丢失阶段。

依赖: 板上的 rknn-toolkit2、OpenCV、NumPy。--live 还需要 source ROS 2 环境。
输出: report.json、候选框图、修复前/修复后 NMS 框图与重建掩码图。
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from rknn.api import RKNN

PROJECT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = PROJECT / "src/scoutcar_perception/models/yolov5_seg/V2.0/seg_V2.0_int8.rknn"
ANCHORS = ((10, 13, 16, 30, 33, 23), (30, 61, 62, 45, 59, 119),
           (116, 90, 156, 198, 373, 326))
LABELS = ("road", "barrier")
BOX_THRESHOLD = 0.6
NMS_THRESHOLD = 0.45
CROP_TOP = 120
MODEL_SIZE = 640
MAX_RESULTS = 128


def capture_live(topic: str, timeout: float) -> np.ndarray:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as error:
        raise RuntimeError("--live 需要先 source ROS 2 环境") from error

    rclpy.init()
    node = Node("seg_frame_analyzer")
    frames = []

    def on_frame(msg: Image) -> None:
        if frames:
            return
        if msg.encoding.lower() not in ("rgb8", "bgr8"):
            raise RuntimeError(f"不支持相机编码: {msg.encoding}")
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = image[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
        frames.append(cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                      if msg.encoding.lower() == "rgb8" else image)

    subscription = node.create_subscription(Image, topic, on_frame,
                                            qos_profile_sensor_data)
    deadline = time.monotonic() + timeout
    try:
        while not frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    if not frames:
        raise RuntimeError(f"{timeout:g} 秒内未收到 {topic} 图像")
    return frames[0]


def read_frame(args: argparse.Namespace) -> np.ndarray:
    if args.live:
        return capture_live(args.topic, args.timeout)
    if args.image:
        frame = cv2.imread(str(args.image))
    else:
        capture = cv2.VideoCapture(str(args.video))
        if not capture.isOpened():
            raise RuntimeError(f"无法打开视频: {args.video}")
        capture.set(cv2.CAP_PROP_POS_FRAMES, args.frame)
        ok, frame = capture.read()
        capture.release()
        if not ok:
            raise RuntimeError(f"无法读取第 {args.frame} 帧")
    if frame is None:
        raise RuntimeError("无法读取图像")
    return frame


def letterbox(frame: np.ndarray) -> tuple[np.ndarray, dict]:
    height, width = frame.shape[:2]
    if height <= CROP_TOP:
        raise RuntimeError(f"图像高度 {height} 不足以裁掉顶部 {CROP_TOP} 行")
    crop_height = height - CROP_TOP
    scale = min(MODEL_SIZE / width, MODEL_SIZE / crop_height)
    resized_width = round(width * scale)
    resized_height = round(crop_height * scale)
    pad_x = (MODEL_SIZE - resized_width) // 2
    pad_y = (MODEL_SIZE - resized_height) // 2
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    canvas = np.full((MODEL_SIZE, MODEL_SIZE, 3), 114, dtype=np.uint8)
    canvas[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = cv2.resize(
        rgb[CROP_TOP:], (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    return canvas, {"width": width, "height": height, "crop_height": crop_height,
                    "pad_x": pad_x, "pad_y": pad_y, "resized_width": resized_width,
                    "resized_height": resized_height,
                    "scale_x": resized_width / width,
                    "scale_y": resized_height / crop_height}


def decode_candidates(outputs: list[np.ndarray]) -> tuple[list[dict], list[np.ndarray]]:
    candidates = []
    coefficients = []
    for head_index, output_index in enumerate((0, 2, 4)):
        det = np.asarray(outputs[output_index][0])
        seg = np.asarray(outputs[output_index + 1][0])
        grid_height, grid_width = det.shape[1:]
        stride = MODEL_SIZE // grid_height
        for anchor_index in range(3):
            base = anchor_index * 7
            locations = np.argwhere(det[base + 4] >= BOX_THRESHOLD)
            for row, col in locations:
                object_score = float(det[base + 4, row, col])
                class_scores = det[base + 5:base + 7, row, col]
                class_id = int(np.argmax(class_scores))
                score = object_score * float(class_scores[class_id])
                if score <= BOX_THRESHOLD:
                    continue
                x = (float(det[base, row, col]) * 2 - 0.5 + col) * stride
                y = (float(det[base + 1, row, col]) * 2 - 0.5 + row) * stride
                box_width = (float(det[base + 2, row, col]) * 2) ** 2 * ANCHORS[head_index][anchor_index * 2]
                box_height = (float(det[base + 3, row, col]) * 2) ** 2 * ANCHORS[head_index][anchor_index * 2 + 1]
                candidate = {"id": len(candidates), "class": LABELS[class_id],
                             "class_id": class_id, "score": round(score, 5),
                             "box_model": [x - box_width / 2, y - box_height / 2,
                                           x + box_width / 2, y + box_height / 2],
                             "head": head_index, "anchor": anchor_index}
                candidates.append(candidate)
                coefficients.append(seg[anchor_index * 32:(anchor_index + 1) * 32,
                                        row, col].astype(np.float32))
    return candidates, coefficients


def overlap(a: list[float], b: list[float]) -> float:
    width = max(0.0, min(a[2], b[2]) - max(a[0], b[0]) + 1.0)
    height = max(0.0, min(a[3], b[3]) - max(a[1], b[1]) + 1.0)
    intersection = width * height
    area_a = (a[2] - a[0] + 1.0) * (a[3] - a[1] + 1.0)
    area_b = (b[2] - b[0] + 1.0) * (b[3] - b[1] + 1.0)
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


def nms(candidates: list[dict], *, corrected: bool) -> tuple[list[int], list[dict]]:
    order = sorted(range(len(candidates)), key=lambda index: candidates[index]["score"], reverse=True)
    classes = sorted({candidate["class_id"] for candidate in candidates})
    removed = []
    for class_id in classes:
        for i in range(len(order)):
            source = order[i]
            # 当前 C++ 实现错误地使用 classIds[i]，而非 classIds[order[i]]。
            source_class = candidates[source]["class_id"] if corrected else candidates[i]["class_id"]
            if source == -1 or source_class != class_id:
                continue
            for j in range(i + 1, len(order)):
                target = order[j]
                target_class = candidates[target]["class_id"] if corrected else candidates[i]["class_id"]
                if target == -1 or target_class != class_id:
                    continue
                iou = overlap(candidates[source]["box_model"], candidates[target]["box_model"])
                if iou > NMS_THRESHOLD:
                    removed.append({"by": source, "removed": target,
                                    "iou": round(iou, 4),
                                    "cross_class": candidates[source]["class_id"] != candidates[target]["class_id"]})
                    order[j] = -1
    return [index for index in order if index != -1][:MAX_RESULTS], removed


def original_box(box: list[float], transform: dict) -> list[int]:
    width, height = transform["width"], transform["height"]
    x1 = round((box[0] - transform["pad_x"]) / transform["scale_x"])
    y1 = round(CROP_TOP + (box[1] - transform["pad_y"]) / transform["scale_y"])
    x2 = round((box[2] - transform["pad_x"]) / transform["scale_x"])
    y2 = round(CROP_TOP + (box[3] - transform["pad_y"]) / transform["scale_y"])
    return [max(0, min(width - 1, x1)), max(CROP_TOP, min(height - 1, y1)),
            max(0, min(width - 1, x2)), max(CROP_TOP, min(height - 1, y2))]


def draw_boxes(frame: np.ndarray, candidates: list[dict], indices: list[int],
               transform: dict) -> np.ndarray:
    image = frame.copy()
    for index in indices:
        candidate = candidates[index]
        x1, y1, x2, y2 = original_box(candidate["box_model"], transform)
        color = (0, 0, 255) if candidate["class_id"] == 1 else (0, 200, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f'{index}:{candidate["class"]} {candidate["score"]:.2f}',
                    (x1, max(16, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
    return image


def make_mask(indices: list[int], candidates: list[dict], coefficients: list[np.ndarray],
              proto: np.ndarray, transform: dict) -> np.ndarray:
    model_mask = np.zeros((MODEL_SIZE, MODEL_SIZE), dtype=np.uint8)
    proto_half = proto.astype(np.float16).astype(np.float32).reshape(32, -1)
    for index in indices:
        candidate = candidates[index]
        coefficient = coefficients[index].astype(np.float16).astype(np.float32)
        low_res = np.where(coefficient @ proto_half > 0, 4, 0).astype(np.uint8).reshape(160, 160)
        full = cv2.resize(low_res, (MODEL_SIZE, MODEL_SIZE), interpolation=cv2.INTER_LINEAR)
        x1, y1, x2, y2 = candidate["box_model"]
        left = max(0, int(np.ceil(x1)))
        top = max(0, int(np.ceil(y1)))
        right = min(MODEL_SIZE, int(np.ceil(x2)))
        bottom = min(MODEL_SIZE, int(np.ceil(y2)))
        if right <= left or bottom <= top:
            continue
        roi = model_mask[top:bottom, left:right]
        value = full[top:bottom, left:right]
        roi[(roi == 0) & (value > 0)] = candidate["class_id"] + 1
    output = np.zeros((transform["height"], transform["width"]), dtype=np.uint8)
    pad_x, pad_y = transform["pad_x"], transform["pad_y"]
    valid = model_mask[pad_y:pad_y + transform["resized_height"],
                       pad_x:pad_x + transform["resized_width"]]
    output[CROP_TOP:] = cv2.resize(valid, (transform["width"], transform["crop_height"]),
                                   interpolation=cv2.INTER_LINEAR)
    return output


def overlay_mask(frame: np.ndarray, mask: np.ndarray) -> np.ndarray:
    image = frame.copy()
    for value, color in ((1, (0, 220, 0)), (2, (0, 0, 255))):
        pixels = mask == value
        image[pixels] = (image[pixels].astype(np.float32) * 0.4 +
                         np.asarray(color, np.float32) * 0.6).astype(np.uint8)
    return image


def summarize(indices: list[int], candidates: list[dict], transform: dict) -> dict:
    result = {}
    for label in LABELS:
        matched = [index for index in indices if candidates[index]["class"] == label]
        result[label] = {"total": len(matched), "left": 0, "right": 0}
        for index in matched:
            x1, _, x2, _ = original_box(candidates[index]["box_model"], transform)
            side = "left" if (x1 + x2) / 2 < transform["width"] / 2 else "right"
            result[label][side] += 1
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--live", action="store_true", help="订阅当前前视相机一帧")
    source.add_argument("--image", type=Path, help="分析图片")
    source.add_argument("--video", type=Path, help="分析录像中的一帧")
    parser.add_argument("--frame", type=int, default=0, help="录像帧号，从 0 开始")
    parser.add_argument("--topic", default="/camera/front/image_raw")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", type=Path,
                        default=Path("/tmp/cityscout_seg_analysis"))
    args = parser.parse_args()
    if not args.model.is_file():
        raise RuntimeError(f"模型文件不存在: {args.model}；请用 --model 指定当前运行的 RKNN 模型")
    frame = read_frame(args)
    model_input, transform = letterbox(frame)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output_dir / "source.png"), frame)

    rknn = RKNN(verbose=False)
    try:
        if rknn.load_rknn(str(args.model)) != 0:
            raise RuntimeError(f"加载模型失败: {args.model}")
        if rknn.init_runtime(target="rk3588", core_mask=RKNN.NPU_CORE_0_1) != 0:
            raise RuntimeError("RKNN 运行环境初始化失败")
        outputs = rknn.inference(inputs=[model_input], data_format=["nhwc"])
        if outputs is None or len(outputs) != 7:
            raise RuntimeError("模型没有返回预期的七路输出")
    finally:
        rknn.release()

    candidates, coefficients = decode_candidates(outputs)
    current, current_removed = nms(candidates, corrected=False)
    corrected, corrected_removed = nms(candidates, corrected=True)
    for candidate in candidates:
        candidate["box_original"] = original_box(candidate["box_model"], transform)
    current_mask = make_mask(current, candidates, coefficients, outputs[6][0], transform)
    corrected_mask = make_mask(corrected, candidates, coefficients, outputs[6][0], transform)
    cv2.imwrite(str(args.output_dir / "candidates.png"),
                draw_boxes(frame, candidates, list(range(len(candidates))), transform))
    for name, indices, mask in (("before_fix", current, current_mask),
                                ("after_fix", corrected, corrected_mask)):
        cv2.imwrite(str(args.output_dir / f"{name}_boxes.png"),
                    draw_boxes(frame, candidates, indices, transform))
        cv2.imwrite(str(args.output_dir / f"{name}_mask.png"), mask)
        cv2.imwrite(str(args.output_dir / f"{name}_overlay.png"),
                    overlay_mask(frame, mask))

    report = {"model": str(args.model), "thresholds": {"box": BOX_THRESHOLD,
              "nms": NMS_THRESHOLD}, "transform": transform,
              "note": "独立 Python RKNN 推理；输出为 float32，边缘数值可能与 C++ INT8 后处理略有差异。",
              "candidates_summary": summarize(list(range(len(candidates))), candidates, transform),
              "before_fix_summary": summarize(current, candidates, transform),
              "after_fix_summary": summarize(corrected, candidates, transform),
              "candidate_boxes": candidates, "before_fix_nms_ids": current,
              "after_fix_nms_ids": corrected,
              "before_fix_removed": current_removed,
              "after_fix_removed": corrected_removed,
              "cross_class_removed_before_fix": [item for item in current_removed if item["cross_class"]],
              "mask_changed_pixels": int(np.count_nonzero(current_mask != corrected_mask))}
    (args.output_dir / "report.json").write_text(json.dumps(report, ensure_ascii=False,
                                                       indent=2) + "\n")
    print(json.dumps({key: report[key] for key in
                      ("candidates_summary", "before_fix_summary", "after_fix_summary",
                       "cross_class_removed_before_fix", "mask_changed_pixels")}, ensure_ascii=False, indent=2))
    print(f"结果目录: {args.output_dir}")


if __name__ == "__main__":
    main()
