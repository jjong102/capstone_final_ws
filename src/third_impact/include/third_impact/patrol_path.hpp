#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace third_impact::patrol_path
{
struct Point { double x, y; };
using Route = std::vector<Point>;

inline double distance(Point a, Point b) { return std::hypot(b.x-a.x, b.y-a.y); }
inline double heading(Point a, Point b) { return std::atan2(b.y-a.y, b.x-a.x); }
inline Point interpolate(Point a, Point b, double t)
{
  return {a.x+t*(b.x-a.x), a.y+t*(b.y-a.y)};
}

class Clearance
{
public:
  Clearance(const nav2_costmap_2d::Costmap2D & map, double half_length, double half_width)
  : map_(map), length_(half_length), width_(half_width), step_(map.getResolution()/2),
    radius_(std::hypot(length_, width_)), cell_pad_(step_*std::sqrt(2.0)+step_/2) {}

  bool pose(Point p, double angle) const
  {
    using namespace nav2_costmap_2d;
    unsigned int x, y, x0, y0, x1, y1;
    const double reach = std::hypot(length_+cell_pad_,width_+cell_pad_);
    if (!map_.worldToMap(p.x,p.y,x,y) ||
      map_.getCost(x,y) >= INSCRIBED_INFLATED_OBSTACLE ||
      !map_.worldToMap(p.x-reach,p.y-reach,x0,y0) ||
      !map_.worldToMap(p.x+reach,p.y+reach,x1,y1))
    {
      return false;
    }
    const double c = std::cos(angle), s = std::sin(angle);
    for (unsigned int iy = y0; iy <= y1; ++iy) {
      for (unsigned int ix = x0; ix <= x1; ++ix) {
        if (map_.getCost(ix,iy) < LETHAL_OBSTACLE) {continue;}
        double wx, wy;
        map_.mapToWorld(ix,iy,wx,wy);
        const double dx = wx-p.x, dy = wy-p.y;
        if (std::abs(c*dx+s*dy) <= length_+cell_pad_ &&
          std::abs(-s*dx+c*dy) <= width_+cell_pad_)
        {
          return false;
        }
      }
    }
    return true;
  }

  bool segment(Point a, Point b, double incoming) const
  {
    const double length = distance(a,b);
    if (length < 1e-6) {return pose(a,incoming);}
    const double angle = heading(a,b);
    const double turn = std::remainder(angle-incoming, 2*M_PI);
    const int rotation_steps = std::max(1,static_cast<int>(std::ceil(std::abs(turn)*radius_/step_)));
    for (int i = 0; i < rotation_steps; ++i) {
      if (!pose(a,incoming+turn*i/rotation_steps)) {return false;}
    }
    const int translation_steps = std::max(1,static_cast<int>(std::ceil(length/step_)));
    for (int i = 0; i <= translation_steps; ++i) {
      if (!pose(interpolate(a,b,static_cast<double>(i)/translation_steps),angle)) {return false;}
    }
    return true;
  }

  bool route(const Route & points, double incoming) const
  {
    for (size_t i = 1; i < points.size(); ++i) {
      if (!segment(points[i-1],points[i],incoming)) {return false;}
      if (distance(points[i-1],points[i]) > 1e-6) {incoming = heading(points[i-1],points[i]);}
    }
    return points.size() >= 2;
  }

private:
  const nav2_costmap_2d::Costmap2D & map_;
  double length_, width_, step_, radius_, cell_pad_;
};

inline Route simplify(const Route & raw, double yaw, const Clearance & check)
{
  if (raw.size() < 2) {return {};}
  Route anchors{raw.front()};
  for (size_t i = 1; i+1 < raw.size(); ++i) {
    if (distance(anchors.back(),raw[i]) >= .08) {anchors.push_back(raw[i]);}
  }
  anchors.push_back(raw.back());

  Route result{anchors.front()};
  // Bound work for long paths or maps where many prospective shortcuts fail.
  size_t checks = 0;
  for (size_t from = 0; from+1 < anchors.size();) {
    size_t to = anchors.size()-1;
    for (; to > from; --to) {
      if (++checks > 128) {return {};}
      if (distance(anchors[from],anchors[to]) > 1e-6 &&
        check.segment(anchors[from],anchors[to],yaw))
      {
        break;
      }
    }
    if (to == from) {return {};}
    yaw = heading(anchors[from],anchors[to]);
    result.push_back(anchors[to]);
    from = to;
  }
  return result;
}

inline Route remaining(const Route & route, Point start)
{
  double closest = std::numeric_limits<double>::infinity();
  size_t segment = 0;
  for (size_t i = 1; i < route.size(); ++i) {
    const auto a = route[i-1], b = route[i];
    const auto length = distance(a,b);
    if (length < 1e-6) {continue;}
    const auto t = std::clamp(((start.x-a.x)*(b.x-a.x)+(start.y-a.y)*(b.y-a.y))/
      (length*length),0.0,1.0);
    const double offset = distance(start,interpolate(a,b,t));
    if (offset <= closest) {
      closest = offset;
      segment = i;
    }
  }
  if (closest > .08) {return {};}
  Route tail{start};
  // Skip an already reached corner to avoid turning back onto it.
  for (size_t i = segment; i < route.size(); ++i) {
    if (i+1 == route.size() || distance(tail.back(),route[i]) >= .05) {
      tail.push_back(route[i]);
    }
  }
  return tail;
}
}  // namespace third_impact::patrol_path
