#include "System.h"
#include "LogMacro.h"

#include <iostream>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>

using Event          = prophesee_event_msgs::Event;               //for prophess
using EventArray     = prophesee_event_msgs::EventArray;
//using Event          = dvs_msgs::Event;                              //for dvs
//using EventArray     = dvs_msgs::EventArray;

std::shared_ptr<EVPose::System> pSystem;
std::string bag_path;
double start_time;

void pubIMUData( std::string &imu_topic, std::shared_ptr<EVPose::System> pSystem){
    //process event data to system
    rosbag::Bag bag;

    bag.open( bag_path, rosbag::bagmode::Read );

    std::vector<std::string> topics{ imu_topic };

    rosbag::View view( bag, rosbag::TopicQuery( topics ) );

    for ( rosbag::MessageInstance const m : rosbag::View( bag ) )
    {
        std::string topic = m.getTopic();
        std::cout<<topic<<std::endl;
        if ( topic == imu_topic )
        {
            sensor_msgs::Imu::ConstPtr ip = m.instantiate<sensor_msgs::Imu>();
            //std::cout<<ip->header.stamp.toSec()<<std::endl;
            pSystem->GrabIMUData(ip->header.stamp.toSec(),
                                 ip->angular_velocity.x, ip->angular_velocity.y, ip->angular_velocity.z,
                                 ip->linear_acceleration.x, ip->linear_acceleration.y, ip->linear_acceleration.z);
            //usleep(5000);
        }
    }

    bag.close();
}

void PubEventData(std::string &event_topic, std::shared_ptr<EVPose::System> pSystem){
  //process event data to system
  rosbag::Bag bag;

  bag.open( bag_path, rosbag::bagmode::Read );

  std::vector<std::string> topics{ event_topic };

  rosbag::View view( bag, rosbag::TopicQuery( topics ) );

  for ( rosbag::MessageInstance const m : rosbag::View( bag ) )
    {
      std::string topic = m.getTopic();

      if ( topic == event_topic )
        {
          EventArray::ConstPtr eap = m.instantiate<EventArray>();
          for ( Event e : eap->events ){
            size_t x = e.x;
            uint16_t y = e.y;
            double   ts = e.ts.toSec();
            bool     p = e.polarity;

              pSystem->GrabEventData( x, y, ts, p );
          }
            while (pSystem->getEventBufferSize()>100000){
                usleep(10000);
            }
        }
    }

  bag.close();
}



int main( int argc, char **argv )
{
  ros::init( argc, argv, "EventDrone_node" );

  ros::NodeHandle nh;

  image_transport::ImageTransport it( nh );
  std::string config_path( argv[1] );

  INFO("[main]", "Read config file at %s", config_path.c_str());
  std::cout << std::fixed << std::setprecision(10);

  pSystem.reset( new EVPose::System(config_path ));

  cv::FileStorage fs( config_path, cv::FileStorage::READ );
  std::string event_topic = fs["event_topic"].string();
  std::string imu_topic   = fs["imu_topic"].string();
  bag_path = fs["bag_path"].string();
  start_time = fs["start_time"];

  //open three threads
  std::thread thd_BackEnd(&EVPose::System::ProcessBackEnd, pSystem);
	
  //reading data from rosbag
  std::thread thd_PubEventData(PubEventData, std::ref(event_topic), pSystem);
  std::thread thd_PubIMUData(pubIMUData, std::ref(imu_topic), pSystem);

  std::thread thd_Draw(&EVPose::System::Draw, pSystem);

  thd_BackEnd.join();
  thd_PubIMUData.join();
  thd_PubEventData.join();
  thd_Draw.join();

  ROS_INFO( "finished..." );

  ros::shutdown();

  return 0;
}