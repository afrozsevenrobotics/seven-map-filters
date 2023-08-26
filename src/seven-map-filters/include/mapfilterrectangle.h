

#pragma once
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
// #include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>

namespace seven_map_filters
{

  class MapFilterRectangle : public costmap_2d::Layer
  {

    public:
      MapFilterRectangle();
      ~MapFilterRectangle();

      virtual void onInitialize();
      // virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
      //                             double* min_x, double* min_y, double* max_x, double* max_y);
      // virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      // virtual void updateRectangle(double length, double width,double rotation, geometry_msgs::Point rectangle_point );
      virtual void getRectanglePoints(double length, double width, double rotation, geometry_msgs::Point rectangle_point,double offset_x, double offset_y, std::vector<geometry_msgs::Point>& rectangle_points);
      virtual void getBoundryPoints(std::vector<geometry_msgs::Point>& boundary_line_points,std::vector<geometry_msgs::Point>& lower_line_points, std::vector<geometry_msgs::Point>& upper_line_points);

      virtual void getBresenhamLineCells(geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<geometry_msgs::Point>& pts);
      virtual void getAllRectangleCell(std::vector<geometry_msgs::Point>lower_line_points, std::vector<geometry_msgs::Point> upper_line_points, std::vector<geometry_msgs::Point> &pts);
      // geometry_msgs::Point rect_p1, rect_p2, rect_p3,rect_p4;
      // void transformRectPoint(geometry_msgs::Point rect_point);

      std::vector<geometry_msgs::Point> pts, lower_line_points, upper_line_points, boundary_line_points;
      std::vector<geometry_msgs::Point> rectangle_points;

      //dynamic reconfigure server
      // std::shared_ptr<dynamic_reconfigure::Server<>> _dsrv;

      double costmap_resolution_;
      std::string global_frame_;
      double length, width, rotation, offset_x, offset_y;
      geometry_msgs::Point recieved_rect_point;

  };  



}
