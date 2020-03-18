#include <ergodic_explore/infomap_tools.h>
#include <boost/thread.hpp>
#include <mutex>
namespace info_map
{
  infomap::infomap()
    : private_nh_("~")
  {
    std::string regions_topic;
    std::string infomap_topic;
    private_nh_.param("regions_topic",regions_topic ,std::string("regions_topic"));
    private_nh_.param("infomap_topic",infomap_topic, std::string("infomap_topic"));
    private_nh_.param("update_frequency",update_frequency_,1.0);
    private_nh_.param("gaussian_variance_inflation",gaussian_variance_inflation,0.1);
    private_nh_.param("quantisation",quantisation,100.0);
    private_nh_.param("scale",scale,1.5);
    subscriber_infopoints_=private_nh_.subscribe<ergodic_explore::ErgInfoMapRegions>(
          regions_topic, 1,
          [this](const ergodic_explore::ErgInfoMapRegions::ConstPtr& msg) {
            regions_callback(msg);
          });
    publisher_infomap_=private_nh_.advertise<nav_msgs::OccupancyGrid>(infomap_topic,1);
    publishtimer= relative_nh_.createTimer(ros::Duration(1. / update_frequency_),
                                           [this](const ros::TimerEvent&) { publish_map(); });

    //allocating map
    map.header.frame_id = "";
    map.header.stamp = ros::Time::now();

  }

  //applying a gaussian PDF
  void infomap::compute_map(){
    ///boost::recursive_mutex mutex_t;
    //std::lock_guard<boost::recursive_mutex> lock(mutex_t);
    //Increasing the width and height of the information density map
    int size_x=scale*regions.info.width;
    int size_y=scale*regions.info.height;
    std::vector<signed char> map_data(size_x*size_y,0);
    double *map_data_double=new double[size_x*size_y];

    double resolution_=regions.info.resolution;

    map.header.frame_id = regions.header.frame_id;
    map.header.stamp = ros::Time::now();
    map.info.map_load_time=ros::Time::now();
    map.info.resolution=resolution_;
    map.info.width=size_x;
    map.info.height=size_y;
    map.info.origin.position.x=scale*regions.info.origin.position.x;
    map.info.origin.position.y=scale*regions.info.origin.position.y;

    ROS_DEBUG("Information Map Creation Started %d %d",size_x,size_y);
    double cum_prob=0.0;
    double max_prob=-1.0;
    for(int j=0;j<size_x*size_y;j++){
      int my=j/size_x;
      int mx=j-my*size_x;
      //0.5 added to evaluate position of centre grid
      double wx = map.info.origin.position.x + (mx + 0.5) * resolution_;
      double wy = map.info.origin.position.y + (my + 0.5) * resolution_;
      int size=0;
      double value=0.0;
      try{
      for(ergodic_explore::Regions reg : regions.data){
          double dist_x = pow((wx-reg.mean_x),2);
          double dist_y = pow((wy-reg.mean_y),2);
          double v_x=reg.var_x+gaussian_variance_inflation;
          double v_y=reg.var_y+gaussian_variance_inflation;
          double posterior= (1/(2*PI*sqrt(v_x)*sqrt(v_y)))
              * exp(-1*((dist_x/(2*v_x))+(dist_y/(2*v_y))));
          value=value+posterior;        
          size++;
      }
      if((value-max_prob)>0.0001){
        max_prob=value;
      }


      map_data_double[j]=value;
      cum_prob += value;
      }
      catch(std::runtime_error e){
        ROS_ERROR("Map Compute Error: %s",e.what());
      }
    }
    for(int j=0;j<size_x*size_y;j++){
      signed char unit_val=static_cast<signed char>(map_data_double[j]/max_prob*quantisation);
      map_data[j]=unit_val;
    }
    ROS_DEBUG("Information Map restructured");
    map.data=map_data;
    publish_map();
    delete(map_data_double);
    ROS_DEBUG("Information Map published: Cum Prob: %lf",cum_prob*pow(resolution_,2));
  }

}//namespace infomap

int main(int argc, char** argv)
{
  ros::init(argc, argv, "infomap");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  info_map::infomap infomap;
  ros::spin();

  return 0;
}
