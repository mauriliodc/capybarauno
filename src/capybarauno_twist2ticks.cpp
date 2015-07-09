//system stuff
#include <string>
#include <stdint.h>
//ros stuff
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "capybarauno/capybara_ticks.h"


using namespace std;

ros::Publisher ticks_publisher;

struct configuration{
    string cmdvel_topic;
    string ticks_topic;
    double left_ticks;
    double right_ticks;
    double baseline;
    int debug;
};

struct configuration c;
uint32_t tick_sequence;


float left_meter_to_ticks,right_meter_to_ticks,baseline;

void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    if(!ros::ok()) return;
    //get absolute speed values, expressed in tick per interval
    float translational_velocity = twist->linear.x;
    float rotational_velocity    = -twist->angular.z*baseline;
    if(c.debug){
        ROS_INFO("LINEAR %f ANGULAR %f ANGULAR AFTER BASELINE %f", twist->linear.x, twist->angular.z,rotational_velocity);
    }
    capybarauno::capybara_ticks ct;
    ct.leftEncoder=(uint16_t)((-translational_velocity+rotational_velocity)/left_meter_to_ticks);
    ct.rightEncoder=(uint16_t)((translational_velocity+rotational_velocity)/right_meter_to_ticks);
    if(c.debug){
        ROS_INFO("TICKS %d %d",ct.leftEncoder, ct.rightEncoder);
    }
    ct.header.stamp=ros::Time::now();
    ct.header.seq=tick_sequence;
    tick_sequence++;
    ticks_publisher.publish(ct);

}


void EchoParameters(){
    printf("%s %s\n","_cmdvel_topic",c.cmdvel_topic.c_str());
    printf("%s %s\n","_ticks_topic",c.ticks_topic.c_str());
    printf("%s %f\n","_left_ticks",c.left_ticks);
    printf("%s %f\n","_right_ticks",c.right_ticks);
    printf("%s %f\n","_baseline",c.baseline);
    printf("%s %d\n","_debug",c.debug);
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "capybarauno_twist2ticks",ros::init_options::AnonymousName);
    ros::NodeHandle n("~");
    n.param<string>("cmdvel_topic", c.cmdvel_topic, "/cmd_vel");
    n.param<string>("tick_topic", c.ticks_topic, "/requested_ticks");
    n.param("left_ticks",c.left_ticks,1.0d);
    n.param("right_ticks",c.right_ticks,1.0d);
    n.param("baseline",c.baseline,1.0d);
    n.param("debug", c.debug, 1);

    EchoParameters();
    left_meter_to_ticks = (float)c.left_ticks;
    right_meter_to_ticks = (float)c.right_ticks;
    baseline= (float)c.baseline;

    ros::Subscriber cmdvel_subscriber = n.subscribe(c.cmdvel_topic.c_str(), 1000, cmdvelCallback);
    ticks_publisher = n.advertise<capybarauno::capybara_ticks>(c.ticks_topic.c_str(), 1000);
    ros::spin();
    return 0;
}




