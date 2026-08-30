#include "road_boundary.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <vector>

namespace road_boundary {
namespace {
struct Run { int left; int right; };
struct Line { float a=0; float b=0; bool valid=false; };
struct CenterPoint { int y; float x; };
struct CenterFit { Line line; int candidates=0; int inliers=0; float confidence=0; };

Line least_squares(const std::vector<CenterPoint> & points,
                   const std::vector<uint8_t> & use) {
  Line line;
  double sy=0,sx=0,syy=0,syx=0;
  int n=0;
  for(size_t i=0;i<points.size();++i) {
    if(!use.empty()&&!use[i]) continue;
    const double y=points[i].y,x=points[i].x;
    sy+=y;sx+=x;syy+=y*y;syx+=y*x;++n;
  }
  const double d=n*syy-sy*sy;
  if(n<2||std::abs(d)<1e-6) return line;
  line.a=static_cast<float>((n*syx-sy*sx)/d);
  line.b=static_cast<float>((sx-line.a*sy)/n);
  line.valid=true;
  return line;
}

CenterFit robust_center_fit(const std::vector<int16_t> & left,
                            const std::vector<int16_t> & right,
                            int width,int height,const Options & o) {
  CenterFit fit;
  const int y0=std::clamp(static_cast<int>(height*o.center_fit_top_ratio),0,height-1);
  const int y1=std::clamp(static_cast<int>(height*o.center_fit_bottom_ratio),y0,height-1);
  std::vector<CenterPoint> points;
  int missing_run=0;
  bool connected=false;
  // 只接受从车前拟合区底部连续向上生长的路面。远端独立块即使完美共线，
  // 也不能作为控制线候选。允许最多 3 行小断裂。
  for(int y=y1;y>=y0;--y) {
    const int l=left[y],r=right[y];
    const bool usable=l>=o.center_edge_margin&&r<width-o.center_edge_margin&&
      r-l+1>=o.center_min_width;
    if(!usable) {
      if(connected&&++missing_run>3) break;
      continue;
    }
    // 必须在底部 8 行内找到入口，否则只是远处悬空块。
    if(!connected&&y<y1-8) return fit;
    connected=true;missing_run=0;
    points.push_back({y,0.5f*(l+r)});
  }
  std::reverse(points.begin(),points.end());
  fit.candidates=static_cast<int>(points.size());
  if(fit.candidates<o.center_min_points) return fit;

  int best_count=0;
  double best_error=std::numeric_limits<double>::max();
  Line best;
  // 枚举距离足够远的点对；单行边缘泄漏无法拉动最优模型。
  for(size_t i=0;i<points.size();++i) for(size_t j=i+1;j<points.size();++j) {
    const int dy=points[j].y-points[i].y;
    if(dy<20) continue;
    const float a=(points[j].x-points[i].x)/dy;
    if(std::abs(a)>1.5f) continue;
    const float b=points[i].x-a*points[i].y;
    int count=0; double error=0;
    for(const auto & p:points) {
      const double e=std::abs(p.x-(a*p.y+b));
      if(e<=o.center_inlier_px) {++count;error+=e;}
    }
    if(count>best_count||(count==best_count&&error<best_error)) {
      best_count=count;best_error=error;best={a,b,true};
    }
  }
  if(!best.valid) return fit;
  std::vector<uint8_t> inlier(points.size(),0);
  for(size_t i=0;i<points.size();++i)
    if(std::abs(points[i].x-(best.a*points[i].y+best.b))<=o.center_inlier_px)
      inlier[i]=1;
  fit.line=least_squares(points,inlier);
  fit.inliers=std::accumulate(inlier.begin(),inlier.end(),0);
  fit.confidence=static_cast<float>(fit.inliers)/fit.candidates;
  if(fit.inliers<o.center_min_points||fit.confidence<o.center_min_inlier_ratio)
    fit.line.valid=false;
  if(fit.line.valid&&std::abs(fit.line.a)>o.center_max_abs_slope)
    fit.line.valid=false;
  return fit;
}

std::vector<Run> find_road_runs(const uint8_t * row, int width, const Options & o) {
  std::vector<Run> runs;
  int first=-1,last=-1;
  for(int x=0;x<width;++x) {
    if(row[x]!=1) continue;
    if(first<0) first=last=x;
    else if(x-last-1<=o.max_inline_gap) last=x;
    else {
      if(last-first+1>=o.min_run_width) runs.push_back({first,last});
      first=last=x;
    }
  }
  if(first>=0&&last-first+1>=o.min_run_width) runs.push_back({first,last});
  return runs;
}

int overlap(const Run & a,const Run & b,int margin) {
  return std::max(0,std::min(a.right+margin,b.right)-std::max(a.left-margin,b.left)+1);
}

const Run * choose_run(const std::vector<Run> & runs,const Run * previous,int anchor) {
  const Run * best=nullptr;
  int best_score=std::numeric_limits<int>::min();
  for(const auto & run:runs) {
    const int center=(run.left+run.right)/2;
    int score=-2*std::abs(center-anchor)+(run.right-run.left+1)/8;
    if(previous) score+=8*overlap(*previous,run,20);
    if(run.left<=anchor&&anchor<=run.right) score+=200;
    if(score>best_score) {best=&run;best_score=score;}
  }
  return best;
}

Line local_line(const std::vector<int16_t> & raw,const std::vector<int> & trusted,int count) {
  Line line;
  const int n=std::min(count,static_cast<int>(trusted.size()));
  if(n<2) return line;
  double sy=0,sx=0,syy=0,syx=0;
  for(int i=static_cast<int>(trusted.size())-n;i<static_cast<int>(trusted.size());++i) {
    const int y=trusted[i]; const int x=raw[y];
    sy+=y;sx+=x;syy+=static_cast<double>(y)*y;syx+=static_cast<double>(y)*x;
  }
  const double d=n*syy-sy*sy;
  if(std::abs(d)<1e-6) return line;
  line.a=static_cast<float>((n*syx-sy*sx)/d);
  line.b=static_cast<float>((sx-line.a*sy)/n);
  line.valid=true;
  return line;
}

void interpolate_gap(std::vector<int16_t> & fitted,std::vector<uint8_t> & repaired,
                     int y0,int x0,int y1,int x1) {
  if(y0==y1) return;
  const int lo=std::min(y0,y1),hi=std::max(y0,y1);
  for(int y=lo;y<=hi;++y) {
    const float t=static_cast<float>(y-y0)/static_cast<float>(y1-y0);
    fitted[y]=static_cast<int16_t>(std::lround(x0+t*(x1-x0)));
    repaired[y]=1;
  }
  fitted[y0]=static_cast<int16_t>(x0); repaired[y0]=0;
  fitted[y1]=static_cast<int16_t>(x1); repaired[y1]=0;
}

void repair_side(const std::vector<int16_t> & raw,int top,int bottom,bool left_side,
                 const Options & o,std::vector<int16_t> & fitted,
                 std::vector<uint8_t> & repaired) {
  fitted=raw; repaired.assign(raw.size(),0);

  // 入口下方连续路面是唯一可学习的基准。后续逐渐外扩不能反过来拖动基准线。
  std::vector<int> entrance;
  for(int y=bottom;y>=top && static_cast<int>(entrance.size())<o.local_fit_points;--y)
    if(raw[y]>=0) entrance.push_back(y);
  const Line reference=local_line(raw,entrance,o.local_fit_points);
  if(!reference.valid) return;

  bool gap=false;
  int anchor_y=-1,anchor_x=-1;
  int reconnect_count=0,reconnect_first_y=-1;
  for(int y=bottom;y>=top;--y) {
    const int predicted=static_cast<int>(std::lround(reference.a*y+reference.b));
    const bool missing=raw[y]<0;
    // 左边界向左、右边界向右才是路口外扩；向内收缩是远处出口候选。
    const bool outward=!missing && (left_side ? raw[y]<predicted-o.boundary_jump_px
                                             : raw[y]>predicted+o.boundary_jump_px);
    if (!gap && (missing || outward)) {
      gap=true; anchor_y=y+1;
      while(anchor_y<=bottom && raw[anchor_y]<0) ++anchor_y;
      if(anchor_y>bottom) return;
      anchor_x=raw[anchor_y]; reconnect_count=0; reconnect_first_y=-1;
    }
    if(!gap) continue;

    fitted[y]=static_cast<int16_t>(predicted); repaired[y]=1;
    // 出口边界必须重新贴近入口参考方向，连续两行才关闭缺口。
    const bool reconnect=!missing && std::abs(raw[y]-predicted)<=o.reconnect_tolerance_px;
    if(reconnect) {
      if(reconnect_count==0) reconnect_first_y=y;
      ++reconnect_count;
      if(reconnect_count>=o.reconnect_rows) {
        interpolate_gap(fitted,repaired,anchor_y,anchor_x,
                        reconnect_first_y,raw[reconnect_first_y]);
        gap=false;
      }
    } else {
      reconnect_count=0; reconnect_first_y=-1;
    }
  }
}

bool any_repaired(const std::vector<uint8_t> & flags,int top,int bottom) {
  return std::any_of(flags.begin()+top,flags.begin()+bottom+1,
                     [](uint8_t value){return value!=0;});
}
}  // namespace

Tracker::Tracker(const Options & options):options_(options) {}
void Tracker::reset() {
  have_center_line_=false;
  previous_center_slope_=0.0f;
  previous_center_intercept_=0.0f;
  center_lost_frames_=0;
}

Result Tracker::estimate(const uint8_t * mask,int width,int height,float near_ratio,
                         uint8_t * processed) {
  Result result;
  if(!mask||width<=0||height<=0) return result;
  if(processed) std::memset(processed,0,static_cast<size_t>(width)*height);
  const int near_y=std::clamp(static_cast<int>(height*near_ratio),0,height-1);
  const int top=std::clamp(static_cast<int>(height*options_.far_row_ratio),0,near_y);
  const int bottom=height-2;
  result.y=near_y;result.fit_top_y=top;result.fit_bottom_y=bottom;
  std::vector<int16_t> raw_l(height,-1),raw_r(height,-1);
  Run previous{}; bool have_previous=false;
  int anchor=width/2;
  for(int y=bottom;y>=top;--y) {
    const auto runs=find_road_runs(mask+static_cast<size_t>(y)*width,width,options_);
    const Run * selected=choose_run(runs,have_previous?&previous:nullptr,anchor);
    if(!selected) continue;
    raw_l[y]=static_cast<int16_t>(selected->left);
    raw_r[y]=static_cast<int16_t>(selected->right);
    previous=*selected;have_previous=true;anchor=(selected->left+selected->right)/2;
  }
  if(!have_previous) {
    if(!have_center_line_||center_lost_frames_>=options_.center_hold_frames) {
      reset();
      return result;
    }
  }

  std::vector<int16_t> fit_l,fit_r;
  std::vector<uint8_t> repair_l,repair_r;
  repair_side(raw_l,top,bottom,true,options_,fit_l,repair_l);
  repair_side(raw_r,top,bottom,false,options_,fit_r,repair_r);

  const CenterFit observed=robust_center_fit(raw_l,raw_r,width,height,options_);
  bool accept=observed.line.valid;
  const float observed_x=observed.line.a*near_y+observed.line.b;
  if(accept&&(observed_x<options_.center_edge_margin||
              observed_x>=width-options_.center_edge_margin)) accept=false;
  if(accept&&have_center_line_) {
    const float previous_x=previous_center_slope_*near_y+previous_center_intercept_;
    if(std::abs(observed_x-previous_x)>options_.center_max_jump_px||
       std::abs(observed.line.a-previous_center_slope_)>options_.center_max_slope_jump)
      accept=false;
  }

  result.center_candidate_count=observed.candidates;
  result.center_inlier_count=observed.inliers;
  result.center_confidence=observed.confidence;
  if(accept) {
    if(have_center_line_) {
      const float alpha=std::clamp(options_.center_smoothing,0.0f,1.0f);
      const float old_x=previous_center_slope_*near_y+previous_center_intercept_;
      const float new_a=(1-alpha)*previous_center_slope_+alpha*observed.line.a;
      const float new_x=(1-alpha)*old_x+alpha*observed_x;
      previous_center_slope_=new_a;
      previous_center_intercept_=new_x-new_a*near_y;
    } else {
      previous_center_slope_=observed.line.a;
      previous_center_intercept_=observed.line.b;
      have_center_line_=true;
    }
    center_lost_frames_=0;
  } else if(have_center_line_&&center_lost_frames_<options_.center_hold_frames) {
    ++center_lost_frames_;
    result.center_predicted=true;
    result.center_confidence=0.0f;
  } else {
    reset();
    return result;
  }

  result.raw_left=raw_l[near_y];result.raw_right=raw_r[near_y];
  if(result.raw_left>=0&&result.raw_right>=0)
    result.raw_width=result.raw_right-result.raw_left+1;
  result.left=fit_l[near_y];result.right=fit_r[near_y];
  if(result.left>=0&&result.right>result.left) result.width=result.right-result.left+1;
  result.center_slope=previous_center_slope_;
  result.center_intercept=previous_center_intercept_;
  result.center_x=static_cast<int>(std::lround(
    result.center_slope*near_y+result.center_intercept));
  result.deviation=result.center_x-width/2;result.valid=true;
  result.left_reconstructed=any_repaired(repair_l,top,bottom);
  result.right_reconstructed=any_repaired(repair_r,top,bottom);
  result.status=(result.left_reconstructed||result.right_reconstructed)
    ?Status::RECONSTRUCTED:Status::NORMAL;

  result.raw_left_boundary.assign(raw_l.begin()+top,raw_l.begin()+bottom+1);
  result.raw_right_boundary.assign(raw_r.begin()+top,raw_r.begin()+bottom+1);
  result.fitted_left_boundary.assign(fit_l.begin()+top,fit_l.begin()+bottom+1);
  result.fitted_right_boundary.assign(fit_r.begin()+top,fit_r.begin()+bottom+1);
  result.left_repaired.assign(repair_l.begin()+top,repair_l.begin()+bottom+1);
  result.right_repaired.assign(repair_r.begin()+top,repair_r.begin()+bottom+1);
  result.center_boundary.reserve(bottom-top+1);
  for(int y=top;y<=bottom;++y) {
    const int x=static_cast<int>(std::lround(result.center_slope*y+result.center_intercept));
    result.center_boundary.push_back(
      static_cast<int16_t>((x>=0&&x<width)?x:-1));
  }
  if(processed) for(int y=top;y<=bottom;++y) {
    if(fit_l[y]>=0) processed[static_cast<size_t>(y)*width+fit_l[y]]=255;
    if(fit_r[y]>=0) processed[static_cast<size_t>(y)*width+fit_r[y]]=255;
  }
  return result;
}

const char * status_name(Status status) {
  switch(status) {
    case Status::NORMAL:return "NORMAL";
    case Status::RECONSTRUCTED:return "RECONSTRUCTED";
    case Status::NO_INFO:return "NO_INFO";
  }
  return "UNKNOWN";
}
}  // namespace road_boundary
