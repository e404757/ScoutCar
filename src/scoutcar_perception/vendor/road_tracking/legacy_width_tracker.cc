#include "legacy_width_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace road_tracking {
namespace {
struct Run { int left; int right; bool from_barrier=false; };
int center(const Run & r) { return (r.left+r.right)/2; }
int overlap(const Run & a,const Run & b,int margin) {
  return std::max(0,std::min(a.right+margin,b.right)-
                    std::max(a.left-margin,b.left)+1);
}

std::vector<Run> find_runs(const uint8_t * row,int width,const LegacyConfig & c) {
  std::vector<Run> runs;
  int left=-1,last=-1;
  for(int x=0;x<width;++x) {
    if(row[x]!=1) continue;
    if(left<0) left=last=x;
    else if(x-last-1<=c.max_inline_gap) last=x;
    else {
      if(last-left+1>=c.min_run_width) runs.push_back({left,last,false});
      left=last=x;
    }
  }
  if(left>=0&&last-left+1>=c.min_run_width) runs.push_back({left,last,false});
  return runs;
}

bool barrier_gap(const uint8_t * row,int width,const LegacyConfig & c,Run & gap) {
  const int cx=width/2;
  int left_edge=-1,right_edge=-1,left=-1,last=-1;
  auto take=[&](int l,int r) {
    if(r-l+1<c.min_run_width) return;
    if((l+r)/2<=cx) left_edge=std::max(left_edge,r);
    else if(right_edge<0||l<right_edge) right_edge=l;
  };
  for(int x=0;x<width;++x) {
    if(row[x]!=2) continue;
    if(left<0) left=last=x;
    else if(x-last-1<=c.max_inline_gap) last=x;
    else {take(left,last);left=last=x;}
  }
  if(left>=0) take(left,last);
  if(left_edge<0||right_edge<0||right_edge-left_edge-1<c.min_run_width) return false;
  gap={left_edge+1,right_edge-1,true};
  return true;
}
}  // namespace

struct LegacyWidthTracker::Estimate {
  bool valid=false;
  int y=0,raw_left=-1,raw_right=-1,left=-1,right=-1;
  bool corrected=false;
};

LegacyWidthTracker::LegacyWidthTracker(const LegacyConfig & config):config_(config) {}
void LegacyWidthTracker::reset() { near_state_=State{};far_state_=State{}; }

LegacyWidthTracker::Estimate LegacyWidthTracker::estimate(
    const uint8_t * mask,int width,int height,const LegacyPreview & p,State & state,
    uint8_t * processed_mask) {
  Estimate out;
  out.y=std::clamp(static_cast<int>(height*p.row_ratio),0,height-1);
  const int cx=width/2;
  const int seed_top=std::max(out.y,static_cast<int>(height*config_.bottom_seed_ratio));
  Run tracked{-1,-1,false};
  bool seeded=false;int seed_y=-1;
  for(int y=height-1;y>=seed_top&&!seeded;--y) {
    auto runs=find_runs(mask+static_cast<size_t>(y)*width,width,config_);
    if(config_.use_barrier_gap) {Run gap;if(barrier_gap(mask+static_cast<size_t>(y)*width,width,config_,gap)) runs.push_back(gap);}
    int best_score=std::numeric_limits<int>::min();
    for(const auto & run:runs) {
      int score=run.right-run.left+1-2*std::abs(center(run)-cx);
      if(run.left<=cx&&cx<=run.right) score+=width;
      if(score>best_score) {best_score=score;tracked=run;seeded=true;seed_y=y;}
    }
  }
  if(!seeded) return out;
  if(processed_mask) {
    const uint8_t * row=mask+static_cast<size_t>(seed_y)*width;
    uint8_t * dst=processed_mask+static_cast<size_t>(seed_y)*width;
    for(int x=tracked.left;x<=tracked.right;++x) if(row[x]==1) dst[x]=255;
  }

  int missing=0;
  std::vector<std::pair<int,int>> samples;
  for(int y=seed_y-1;y>=out.y;--y) {
    auto runs=find_runs(mask+static_cast<size_t>(y)*width,width,config_);
    if(config_.use_barrier_gap) {Run gap;if(barrier_gap(mask+static_cast<size_t>(y)*width,width,config_,gap)) runs.push_back(gap);}
    int best=-1,best_overlap=0;
    for(size_t i=0;i<runs.size();++i) {
      const int score=overlap(tracked,runs[i],config_.overlap_margin);
      if(score>best_overlap) {best_overlap=score;best=static_cast<int>(i);}
    }
    if(best<0) {if(++missing>config_.max_missing_rows) return out;continue;}
    missing=0;tracked=runs[best];
    if(processed_mask) {
      const uint8_t * row=mask+static_cast<size_t>(y)*width;
      uint8_t * dst=processed_mask+static_cast<size_t>(y)*width;
      for(int x=tracked.left;x<=tracked.right;++x) if(row[x]==1) dst[x]=255;
    }
    if(y<=out.y+100&&y>=out.y+12) samples.emplace_back(y,center(tracked));
  }
  out.raw_left=tracked.left;out.raw_right=tracked.right;
  const int raw_width=out.raw_right-out.raw_left+1;
  if(raw_width<p.min_width) return out;

  int predicted=center(tracked);
  if(samples.size()>=8) {
    double sy=0,sx=0,syy=0,syx=0;
    for(const auto & s:samples) {sy+=s.first;sx+=s.second;syy+=double(s.first)*s.first;syx+=double(s.first)*s.second;}
    const double n=samples.size(),d=n*syy-sy*sy;
    if(std::abs(d)>1e-6) predicted=std::clamp(static_cast<int>(std::lround(
      ((n*syx-sy*sx)/d)*out.y+(sx-((n*syx-sy*sx)/d)*sy)/n)),0,width-1);
  }
  out.left=out.raw_left;out.right=out.raw_right;
  if(raw_width>p.max_width) {
    bool keep_left;
    if(state.valid) {
      const int dl=std::abs(out.raw_left-state.left),dr=std::abs(out.raw_right-state.right);
      if(dl<=config_.boundary_stable&&dr>config_.boundary_stable) keep_left=true;
      else if(dr<=config_.boundary_stable&&dl>config_.boundary_stable) keep_left=false;
      else keep_left=dl<=dr;
    } else {
      const int expected_left=predicted-p.expected_width/2;
      const int expected_right=expected_left+p.expected_width-1;
      keep_left=std::abs(out.raw_left-expected_left)<=std::abs(out.raw_right-expected_right);
    }
    if(keep_left) out.right=std::min(width-1,out.left+p.expected_width-1);
    else out.left=std::max(0,out.right-p.expected_width+1);
    out.corrected=true;
  }
  out.valid=true;state={true,out.left,out.right};
  return out;
}

road_boundary::Result LegacyWidthTracker::process(const uint8_t * mask,int width,int height,
                                                  uint8_t * processed_mask) {
  const size_t mask_size=static_cast<size_t>(width)*height;
  if(processed_mask) std::fill(processed_mask,processed_mask+mask_size,0);
  const Estimate near=estimate(mask,width,height,config_.near,near_state_,processed_mask);
  // f977e72 底层已对过宽路面做了单侧宽度修正，修正成功后应直接采用。
  // 旧调用层再用 raw_width>max 丢弃它会使宽度修正永远无法生效。
  const bool near_bad=!near.valid||near.raw_left<=config_.near.edge_margin||
    near.raw_right>=width-1-config_.near.edge_margin;
  Estimate chosen=near;
  if(near_bad) {
    std::vector<uint8_t> far_mask;
    if(processed_mask) far_mask.resize(mask_size,0);
    const Estimate far=estimate(mask,width,height,config_.far,far_state_,
                                far_mask.empty()?nullptr:far_mask.data());
    const bool far_ok=far.valid&&
      far.raw_left>config_.far.edge_margin&&far.raw_right<width-1-config_.far.edge_margin;
    if(far_ok) chosen=far; else chosen=Estimate{};
    if(processed_mask&&!far_mask.empty())
      std::copy(far_mask.begin(),far_mask.end(),processed_mask);
  }

  road_boundary::Result result;
  result.y=chosen.y;result.fit_top_y=chosen.y;result.fit_bottom_y=height-2;
  if(!chosen.valid) return result;
  result.valid=true;result.raw_left=chosen.raw_left;result.raw_right=chosen.raw_right;
  result.raw_width=chosen.raw_right-chosen.raw_left+1;
  result.left=chosen.left;result.right=chosen.right;result.width=chosen.right-chosen.left+1;
  result.center_x=(chosen.left+chosen.right)/2;
  result.deviation=result.center_x-width/2;
  result.center_slope=0;result.center_intercept=result.center_x;
  result.center_confidence=1;result.center_candidate_count=1;result.center_inlier_count=1;
  result.status=chosen.corrected?road_boundary::Status::RECONSTRUCTED:road_boundary::Status::NORMAL;
  result.left_reconstructed=chosen.corrected;result.right_reconstructed=chosen.corrected;
  result.center_boundary.assign(height-1-chosen.y,static_cast<int16_t>(result.center_x));
  return result;
}

}  // namespace road_tracking
