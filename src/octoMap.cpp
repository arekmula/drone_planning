#include <ros/ros.h>
#include <drone_planning/conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>


void octomap_listener(const octomap_msgs::Octomap &octMap)
{
//    for(int i=0; i < octMap.data.size(); i++){
//        std::cout << int(octMap.data[i]) << "\n";
//    }
    std::cout << "octMap Resolution: " << octMap.resolution << "\n";
    std::cout << "octMap Binary: " << octMap.binary <<"\n";
    std::cout << "octMap id: " << octMap.id << "\n";
}

void octomapCloud_listener(const sensor_msgs::PointCloud2 &cloud){

//    std::cout<<int(cloud.data[0])<< "\n";

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "nswr_gps");
  ros::NodeHandle node("~");


  // GPS Fix subscribers
  ros::Subscriber octomapFull_sub = node.subscribe("/octomap_full", 10, octomap_listener);
  ros::Subscriber octmapPointCloud_sub = node.subscribe("/octomap_point_cloud_centers", 10, octomapCloud_listener);



  ros::spin();
}
