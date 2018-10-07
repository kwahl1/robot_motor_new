
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <phidgets/motor_encoder.h>
#include <pid/pid.h>
//#include <Float64.h>


class MotorControllerNode
{
public:
    ros::NodeHandle n;

    ros::Publisher pub_motor_input;
    ros::Publisher pub_controller_state;

    ros::Subscriber sub_motor_encoder;
    ros::Subscriber sub_controller_output;


    MotorControllerNode(){
        n = ros::NodeHandle("~");

        control_frequency = 25.0;
        wheel_radius = 0.097/2.0;
        base = 0.209;
        ticks_per_rev = 890.0;

        delta_encoder = 0.0;
        prev_encoder = 0.0;
        count_encoder = 0.0;

        //remap all topic names
        pub_motor_input = n.advertise<std_msgs::Float32>("cmd_vel", 1);
        pub_controller_state= n.advertise<std_msgs::Float64>("state", 1);

        sub_controller_output = n.subscribe("control_effort",1,&MotorControllerNode::controlEffortCallback,this);
        sub_motor_encoder = n.subscribe("encoder",1,&MotorControllerNode::encoderCallback,this);

    }

    ~MotorControllerNode(){

    }


    void controlEffortCallback(const std_msgs::Float64::ConstPtr& msg)
    {
      control_effort = (float) msg->data;

    }

    void encoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
    {
        count_encoder = (float) msg->count;
        delta_encoder = count_encoder-prev_encoder;
        prev_encoder = count_encoder;
        velocity = (double) (2*M_PI*wheel_radius*control_frequency*delta_encoder)/ticks_per_rev;
    }


    float getControlFrequency()
    {
        return control_frequency;
    }


    void updateController(){

      state_msg.data = velocity;
      control_effort_msg.data = control_effort;

      pub_controller_state.publish(state_msg);
      pub_motor_input.publish(control_effort_msg);

    }

private:


    std_msgs::Float64 state_msg;
    std_msgs::Float32 control_effort_msg;
    ros::Time  vel_time;

    double control_frequency;

    float count_encoder;
    float prev_encoder;
    float delta_encoder;
    double velocity;

    float wheel_radius;
    float base;
    float ticks_per_rev;

    float control_effort;
};



int main(int argc, char **argv)
{

  ros::init(argc, argv, "motor_controller");
  MotorControllerNode motor_controller;

  ros::Rate loop_rate(motor_controller.getControlFrequency());

  while(ros::ok())
  {
      motor_controller.updateController();
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
