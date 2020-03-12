#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// Based hand-over-fist on some ROS template somewhere

class VESCTranslator
{

public:
    VESCTranslator()
    {
        //Topics to publish
        left_pub = n.advertise<std_msgs::Float64>("left_duty_cycle", 10);
        right_pub = n.advertise<std_msgs::Float64>("right_duty_cycle", 10);

        //Topic to subscribe
        sub = n.subscribe("control", 10, &VESCTranslator::twistCallback, this);

        //Timer setup
        timer = n.createTimer(ros::Duration(0.1), &VESCTranslator::timerCallback, this);
    }

private:
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
      double speedMultiplier = 0.5; //Limits duty cycle for left and right speeds
      double leftSpeed = (msg->linear.x - msg->angular.z) * speedMultiplier;
      double rightSpeed = (msg->linear.x + msg->angular.z) * speedMultiplier;

      left_msg.data = leftSpeed;
      right_msg.data = rightSpeed;
    }

    void timerCallback(const ros::TimerEvent&)
    {
      left_pub.publish(left_msg);
      right_pub.publish(right_msg);
    }

    std_msgs::Float64 left_msg;
    std_msgs::Float64 right_msg;

    ros::NodeHandle n;
    ros::Publisher left_pub;
    ros::Publisher right_pub;
    ros::Subscriber sub;
    ros::Timer timer;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vesc_translator");
  VESCTranslator TranslatorObject;
  ros::spin();

  return 0;
}
