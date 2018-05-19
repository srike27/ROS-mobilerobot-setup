/*
 The current navigation stack only accepts sensor data published either using sensor_msgs::PointCloud or sensor_msgs::LaserScan
 These messages contain time and frame dependent information and hence need to follow a certain format
*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);

  unsigned int num_points = 100;

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    sensor_msgs::PointCloud cloud; // create the object to send data with
    cloud.header.stamp = ros::Time::now(); // set timestamp of message
    cloud.header.frame_id = "sensor_frame"; // set frame after tf transform for message

    cloud.points.resize(num_points); //set the number of points to which pointcloud will be generated

    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);  // sets number of channels
    cloud.channels[0].name = "intensities"; // names the channel as intensities
    cloud.channels[0].values.resize(num_points); // sets the size of the data to be stored in the channels

    //generate some fake data for our point cloud by setting x,y,z and values of the channel of the corresponding
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = 1 + count;
      cloud.points[i].y = 2 + count;
      cloud.points[i].z = 3 + count;
      cloud.channels[0].values[i] = 100 + count;
    }

    cloud_pub.publish(cloud); // publish the data to the topic
    ++count;
    r.sleep(); // makes sure that the data is synchronous
  }
}

