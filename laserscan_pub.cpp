/*
 The current navigation stack only accepts sensor data published either using sensor_msgs::PointCloud or sensor_msgs::LaserScan
 These messages contain time and frame dependent information and hence need to follow a certain format
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;// create the object to send data with
    scan.header.stamp = scan_time;// set timestamp of message
    scan.header.frame_id = "laser_frame";// set frame after tf transform for message
    scan.angle_min = -1.57; // set the start angle of the laserscan
    scan.angle_max = 1.57; // set the end angle of the laserscan
    scan.angle_increment = 3.14 / num_readings; // set the distance between any two scan points
    scan.time_increment = (1 / laser_frequency) / (num_readings); //set the time between 2 scans
    scan.range_min = 0.0; // set the min range of hardware in metres
    scan.range_max = 100.0; // set the max range of hardware in metres

    scan.ranges.resize(num_readings); // set the size of the array of range
    scan.intensities.resize(num_readings); // set the size of array of intensities
//generate some fake data for our laser scan by setting range and intensity
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i]; 
      scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan); //publish the data to the topic
    ++count;
    r.sleep();// makes sure that the data is synchronous
  }
}
