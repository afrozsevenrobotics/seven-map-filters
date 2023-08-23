#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>

namespace seven_map_filters
{

class MapFilterRectangle : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  MapFilterRectangle();

  virtual void onInitialize();
  virtual void updateRectangle(double length, double width,double rotation, geometry_msgs::Point rectangle_point );
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  
};


}
geometry_msgs::Point rect_p1, rect_p2, rect_p3,rect_p4;

void transformRectPoint(geometry_msgs::Point rect_point);