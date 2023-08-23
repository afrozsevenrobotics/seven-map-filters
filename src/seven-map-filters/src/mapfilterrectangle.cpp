#include <mapfilterrectangle.h>




// void seven_map_filters::onInitialize(){

// }
void transformRectPoint( geometry_msgs::Point rect_point, double rotation, std::vector<geometry_msgs::Point> &rectangle_points){
        
        tf2::Vector3 v;
        tf2::Quaternion q;
        q.setRPY(0,0,rotation);
        geometry_msgs::Quaternion g_q;
        // geometry_msgs::VectorStamped vec(v);
        // tf2::Vector3 v_new;
        // v_new = tf2::quatRotate(q,v);
        g_q = tf2::toMsg(q);
        geometry_msgs::TransformStamped transform;
        transform.transform.rotation = g_q;
        geometry_msgs::Point new_point;
        tf2::doTransform(rect_point,new_point,transform);
        rectangle_points.push_back(new_point);
        

}

// void seven_map_filters::MapFilterRectangle::updateRectangle(double length, double width, double rotation, geometry_msgs::Point rectangle_point){

//     rect_p1 = rect_p2 = rect_p3 = rect_p4 = rectangle_point;
//     rect_p2.x = rectangle_point.x + length;
//     rect_p3.x = rectangle_point.x + length;
//     rect_p3.y = rectangle_point.y + width;
//     rect_p4.y = rectangle_point.y + width;
//     transformRectPoint(rect_p2,rotation);
//     transformRectPoint(rect_p3,rotation);
//     transformRectPoint(rect_p4,rotation);
    
// }

void updateRectangle(double length, double width, double rotation, geometry_msgs::Point rectangle_point,std::vector<geometry_msgs::Point>& rectangle_points){

    rect_p1 = rect_p2 = rect_p3 = rect_p4 = rectangle_point;
    rect_p2.x = rectangle_point.x + length;
    rect_p3.x = rectangle_point.x + length;
    rect_p3.y = rectangle_point.y + width;
    rect_p4.y = rectangle_point.y + width;
    rectangle_points.push_back(rect_p1);
    transformRectPoint(rect_p2,rotation,rectangle_points);
    transformRectPoint(rect_p3,rotation,rectangle_points);
    transformRectPoint(rect_p4,rotation,rectangle_points);
    
}


void getBresenhamLineCells(geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<geometry_msgs::Point>& pts) {
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
 
void findAllRectangleCell(std::vector<geometry_msgs::Point>lower_line_points, std::vector<geometry_msgs::Point> upper_line_points, std::vector<geometry_msgs::Point> &pts){
    int iterate_size = upper_line_points.size();
    if(lower_line_points.size() > upper_line_points.size()){
        iterate_size = lower_line_points.size();
    }
    
    for(int i = 0; i < iterate_size; i++){
        getBresenhamLineCells(lower_line_points[i],upper_line_points[i],pts);
    }
}

int main(){

geometry_msgs::Point p;
p.x = -5.6;
p.y = -0.4;
p.z = 0;
double length = 5.1;
double width = 5.1;
double rotation = 3.14; //rotation should be in radian
std::vector<geometry_msgs::Point> rectangle_points;
std::vector<geometry_msgs::Point> lower_line_points, upper_line_points;
std::vector<geometry_msgs::Point> pts;
updateRectangle(length,width,rotation,p,rectangle_points);
getBresenhamLineCells(rectangle_points[0],rectangle_points[1],lower_line_points);
getBresenhamLineCells(rectangle_points[3],rectangle_points[2],upper_line_points);

// std::cout<<"lower_line_points->"<<lower_line_points.size()<<std::endl;
// std::cout<<"upper_line_points->"<<upper_line_points.size()<<std::endl;
// std::cout<<p.x<<"    " << p.y<<std::endl;
findAllRectangleCell(lower_line_points,upper_line_points,pts);
    std::cout<<"printing value"<<pts.size()<<std::endl;
for(int i = 0; i < pts.size(); i++){
    std::cout << pts[i].x <<"  "<<pts[i].y<<std::endl;
}
    return 0;
}
