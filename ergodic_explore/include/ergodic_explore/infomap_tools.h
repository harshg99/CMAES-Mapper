#ifndef INFOMAP_TOOLS_H
#define INFOMAP_TOOLS_H

#define PI 3.14159

#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <ergodic_explore/ErgInfoMapRegions.h>
#include <ergodic_explore/Regions.h>
#include <costmap_2d/costmap_2d.h>
#include <math.h>

namespace info_map
{
class infomap{
public:

  infomap();

  /**
   * @brief callback for receieved messages
   * @return
   */
  void regions_callback(const ergodic_explore::ErgInfoMapRegions::ConstPtr& data){
    regions.header=data->header;
    regions.info=data->info;
    regions.data=data->data;
    compute_map();
  }

  /**
   * @brief computes the information density map
   * @return
   */
  void compute_map();

  /**
   * @brief publishes the information density map
   * @return
   */
  void publish_map(){
    publisher_infomap_.publish(map);
  }

private:
  ros::Subscriber subscriber_infopoints_;
  ros::Publisher  publisher_infomap_;
  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  ergodic_explore::ErgInfoMapRegions regions;
  nav_msgs::OccupancyGrid map;

  ros::Timer publishtimer;
  double update_frequency_;
  double gaussian_variance_inflation;
  double quantisation,scale;

};
}

// Subscribes to an information topic multiplexer
// Generates an information density map for the planner
// Optional: Make the information density on the fly
#endif // INFOMAP_TOOLS_H
