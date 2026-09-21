#!/bin/bash

set -u

empty_media="/dev/media2"
hidden_media="/dev/media2.rkaiq-disabled"
rkaiq_pid=""

restore_media_node() {
    if [[ -e "${hidden_media}" && ! -e "${empty_media}" ]]; then
        mv "${hidden_media}" "${empty_media}"
    fi
}

stop_rkaiq() {
    restore_media_node
    if [[ -n "${rkaiq_pid}" ]] && kill -0 "${rkaiq_pid}" 2>/dev/null; then
        kill "${rkaiq_pid}"
        wait "${rkaiq_pid}" 2>/dev/null || true
    fi
}

trap stop_rkaiq EXIT INT TERM

# RKAIQ 5.0 crashes when it initializes the enabled ISP whose CAM1 sensor is
# absent. Hide that empty media node only while RKAIQ enumerates the cameras.
restore_media_node
if [[ -e "${empty_media}" ]]; then
    mv "${empty_media}" "${hidden_media}"
fi

/usr/bin/rkaiq_3A_server &
rkaiq_pid=$!

sleep 2
restore_media_node

wait "${rkaiq_pid}"
