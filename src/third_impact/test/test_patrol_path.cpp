#include <iostream>
#include <stdexcept>

#include "third_impact/patrol_path.hpp"

using namespace third_impact::patrol_path;

void require(bool ok, const char * message)
{
  if (!ok) {throw std::runtime_error(message);}
}

int main()
{
  using namespace nav2_costmap_2d;
  Costmap2D map(120,120,.05,-3.,-3.,FREE_SPACE);
  const Clearance check(map,.36,.23);
  const Route raw{{-2.,0.},{-1.2,.3},{-.6,.7},{0.,.9},{.6,.7},{1.2,.3},{2.,0.}};
  // Facing opposite the departure is fine if there is space for the turn.
  require(simplify(raw,M_PI,check).size()==2,"Open-space curve did not become a straight path");

  for (unsigned int y = 48; y < 70; ++y) {
    map.setCost(59,y,LETHAL_OBSTACLE);
    map.setCost(60,y,LETHAL_OBSTACLE);
  }
  require(!check.segment(raw.front(),raw.back(),0.),"Shortcut crosses a wall");
  const auto around = simplify(raw,0.,check);
  require(around.size()>2 && around.size()<raw.size(),"Safe detour was not simplified");
  require(check.route(around,0.),"Shortened detour has an unsafe segment or turn");

  map.setCost(60,78,NO_INFORMATION);
  require(!check.segment({-.8,.9},{.8,.9},0.),"Shortcut crosses unknown space");
  const auto tail = remaining(around,interpolate(around[0],around[1],.5));
  require(tail.size()>=2 && distance(tail.back(),raw.back())<1e-6,"Replan lost its saved goal");
  require(remaining(around,{-2.9,-2.9}).empty(),"Replan reused an unrelated route");

  Costmap2D corridor(120,120,.05,-3.,-3.,FREE_SPACE);
  for (unsigned int x=0; x<120; ++x) {
    corridor.setCost(x,53,LETHAL_OBSTACLE);
    corridor.setCost(x,66,LETHAL_OBSTACLE);
  }
  const Clearance narrow(corridor,.36,.23);
  require(narrow.segment({-1.,0.},{1.,0.},0.),"Aligned footprint should fit");
  require(!narrow.segment({-1.,0.},{1.,0.},M_PI),"Swept turn leaves the corridor");
  std::cout << "Patrol straightening and footprint checks passed\n";
}
