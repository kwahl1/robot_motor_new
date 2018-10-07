#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>




class SetpointNode
{
public:
    ros::NodeHandle n;
    ros::Publisher pub_left_setpoint;
    ros::Publisher pub_right_setpoint;
    ros::Subscriber sub_twist;


    SetpointNode()
    {
        n = ros::NodeHandle("~");

        control_frequency = 10.0;
        wheel_radius = 0.0352;
        base = 0.23;

        desired_w_left = 0;
        desired_w_right = 0;

        pub_left_setpoint = n.advertise<std_msgs::Float64>("/left_motor/setpoint", 10);
        pub_right_setpoint = n.advertise<std_msgs::Float64>("/right_motor/setpoint", 10);

        sub_twist = n.subscribe<geometry_msgs::Twist>("/motor_controller/twist",10,&SetpointNode::twistCallback,this);

    }



    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {

      linear_x = (float) msg->linear.x;
      angular_z = (float) msg->angular.z;

      desired_w_left = (double) (linear_x-0.5*base*angular_z);
      desired_w_right = (double) (linear_x+0.5*base*angular_z);

      msg_left.data = desired_w_left;
      msg_right.data = desired_w_right;

      pub_right_setpoint.publish(msg_left);
      pub_left_setpoint.publish(msg_right);

    }


    float getControlFrequency()
    {
        return control_frequency;
    }


private:
    std_msgs::Float64 msg_left;
    std_msgs::Float64 msg_right;

    float wheel_radius;
    float base;
    double control_frequency;

    float linear_x;
    float angular_z;
    double desired_w_left;
    double desired_w_right;

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "setpoint_node");
  SetpointNode setpoint;
  ros::Rate loop_rate(setpoint.getControlFrequency());

  while(ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

