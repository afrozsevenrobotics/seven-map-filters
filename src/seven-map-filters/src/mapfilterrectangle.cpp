
#include <mapfilterrectangle.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(seven_map_filters::MapFilterRectangle,costmap_2d::Layer)

namespace seven_map_filters{
  
  MapFilterRectangle::MapFilterRectangle(){}

  MapFilterRectangle::~MapFilterRectangle(){

  }

  void MapFilterRectangle::onInitialize(){
      ros::NodeHandle nh("~/" + name_);
      current_ = true;

      costmap_resolution_ = layered_costmap_->getCostmap()->getResolution();
      global_frame_ = layered_costmap_->getGlobalFrameID();
      


  }

  void MapFilterRectangle::getRectanglePoints(double length, double width, double rotation, geometry_msgs::Point rectangle_point,double offset_x, double offset_y, std::vector<geometry_msgs::Point>& rectangle_points){
        
        double cos_value = std::cos(rotation);
        double sin_value = std::sin(rotation);
        geometry_msgs::Point p1,p2, p3, p4;
        p1 = rectangle_point;
        p2.x = offset_x + rectangle_point.x + cos_value * length;
        p2.y = offset_y + rectangle_point.y + sin_value * length;
        
        p3.x = offset_x + rectangle_point.x + (cos_value * length) - (sin_value * width);
        p3.y = offset_y + rectangle_point.y + (sin_value * length) + (cos_value * width);

        p4.x = offset_x + rectangle_point.x - (sin_value * width);
        p4.y = offset_y + rectangle_point.y + (cos_value * width);
        rectangle_points.push_back(p1);
        rectangle_points.push_back(p2);
        rectangle_points.push_back(p3);
        rectangle_points.push_back(p4);

  }

  void MapFilterRectangle::getBoundryPoints(std::vector<geometry_msgs::Point>&boundary_points,std::vector<geometry_msgs::Point>&lower_line_points,std::vector<geometry_msgs::Point>&upper_line_points){
        boundary_line_points.clear();
        lower_line_points.clear();
        rectangle_points.clear();
        getRectanglePoints(length,width,rotation,recieved_rect_point,offset_x,offset_y,rectangle_points);
        getBresenhamLineCells(rectangle_points[0],rectangle_points[1],lower_line_points);
        getBresenhamLineCells(rectangle_points[3],rectangle_points[2],upper_line_points);
  }

  void MapFilterRectangle::getBresenhamLineCells(geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<geometry_msgs::Point>& pts) {
    //Bresenham Ray-Tracing
    int x0 = p1.x;
    int x1 = p2.x;
    int y0 = p1.y;
    int y1 = p2.y;
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel
  
    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;
  
    geometry_msgs::Point pt;
  
    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }
  
    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }
  
    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator 
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }
  
    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
      pt.x = x;      //Draw the current pixel
      pt.y = y;
      pts.push_back(pt);
  
      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
      {
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }
  }
  
  void MapFilterRectangle::getAllRectangleCell(std::vector<geometry_msgs::Point>lower_line_points, std::vector<geometry_msgs::Point> upper_line_points, std::vector<geometry_msgs::Point> &pts){
      int iterate_size = upper_line_points.size();
      if(lower_line_points.size() > upper_line_points.size()){
          iterate_size = lower_line_points.size();
      }
      
      for(int i = 0; i < iterate_size; i++){
          getBresenhamLineCells(lower_line_points[i],upper_line_points[i],pts);
      }
  }

}



int main(){

// geometry_msgs::Point p;
// p.x = 2;
// p.y = 0;
// p.z = 0;
// double length = 5;
// double width = 5;
// double rotation = 3.14; //rotation should be in radian
// double offset_x = 0.0;
// double offset_y = 0.0;
// std::vector<geometry_msgs::Point> rectangle_points;
// std::vector<geometry_msgs::Point> lower_line_points, upper_line_points;
// std::vector<geometry_msgs::Point> pts;

// // updateRectangle(length,width,rotation,p,rectangle_points);
// findRectanglePoints(length,width,rotation,p,offset_x,offset_y,rectangle_points);
// getBresenhamLineCells(rectangle_points[0],rectangle_points[1],lower_line_points);
// getBresenhamLineCells(rectangle_points[3],rectangle_points[2],upper_line_points);
// findAllRectangleCell(lower_line_points,upper_line_points,pts);

// for(int i = 0; i < pts.size(); i++){
//   std::cout<<"P"<<i<<" = ("<<pts[i].x <<" , "<<pts[i].y<<" )"<<std::endl;
// }
    return 0;
}
