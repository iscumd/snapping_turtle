#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

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
    }

private:
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
      int speedMultiplier = 1;
      double leftSpeed = (msg->linear.x - msg->angular.z) * speedMultiplier;
      double rightSpeed = (msg->linear.x + msg->angular.z) * speedMultiplier;
      std_msgs::Float64 left_msg;
      std_msgs::Float64 right_msg;
      left_msg.data = leftSpeed;
      right_msg.data = rightSpeed;
      left_pub.publish(left_msg);
      right_pub.publish(right_msg);


    }
    ros::NodeHandle n;
    ros::Publisher left_pub;
    ros::Publisher right_pub;
    ros::Subscriber sub;
};



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "vesc_translator");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */


  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  VESCTranslator TranslatorObject;
  ros::spin();

  return 0;
}
