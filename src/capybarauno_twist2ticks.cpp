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
    int left_meter_to_ticks;
    int right_meter_to_ticks;
    int debug;
};

struct configuration c;
uint32_t tick_sequence;

void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    if(!ros::ok()) return;
    //get absolute speed values, expressed in tick per interval
    int translational_velocity = twist->linear.x;
    int rotational_velocity    = twist->angular.z;

    capybarauno::capybara_ticks ct;
    ct.leftEncoder=-translational_velocity+rotational_velocity;
    ct.rightEncoder=translational_velocity+rotational_velocity;
    ct.header.stamp=ros::Time::now();
    ct.header.seq=tick_sequence;
    tick_sequence++;
    ticks_publisher.publish(ct);

}


void EchoParameters(){
    printf("%s %s\n","_cmdvel_topic",c.cmdvel_topic.c_str());
    printf("%s %s\n","_ticks_topic",c.ticks_topic.c_str());
    printf("%s %d\n","_left_meter_to_ticks",c.left_meter_to_ticks);
    printf("%s %d\n","_right_meter_to_ticks",c.right_meter_to_ticks);
    printf("%s %d\n","_debug",c.debug);
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "capybarauno_twist2ticks",ros::init_options::AnonymousName);
    ros::NodeHandle n("~");
    n.param<string>("cmdvel_topic", c.cmdvel_topic, "/cmd_vel");
    n.param<string>("tick_topic", c.ticks_topic, "/requested_ticks");
    n.param("left_meter_to_ticks",c.left_meter_to_ticks,1);
    n.param("right_meter_to_ticks",c.right_meter_to_ticks,1);
    n.param("debug", c.debug, 1);

    EchoParameters();


    ros::Subscriber cmdvel_subscriber = n.subscribe(c.cmdvel_topic.c_str(), 1000, cmdvelCallback);
    ticks_publisher = n.advertise<capybarauno::capybara_ticks>(c.ticks_topic.c_str(), 1000);
    ros::spin();
    return 0;
}




