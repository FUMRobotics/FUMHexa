#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <std_msgs/Int16.h>
#include <math.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hexa_node2");
	
	ros::NodeHandle n;
	
	ros::Publisher chatter_pub0 = n.advertise<std_msgs::Int16>("desiredCurrent0", 1000);
	ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int16>("desiredCurrent1", 1000);
	
	int count = 0;
	
	ros::Rate loop_rate(200);
	while (ros::ok()){

	  std_msgs::Int16 msg0;
	  std_msgs::Int16 msg1;
   
	  msg0.data = -800 * sin(count / 120.00); 
	  msg1.data = 800 * sin(count / 120.00);   
       

	//msg0.data = -100 * count % 4; 
	//msg1.data = 00 * count % 4; 
	  chatter_pub0.publish(msg0);
	  chatter_pub1.publish(msg1);

		
	  count ++;
	  ros::spinOnce();
	  loop_rate.sleep();
	}
  
  
    return 0;
}
