//system stuff
#include <string>
#include <stdint.h>
//ros stuff
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "capybarauno/capybara_ticks.h"


using namespace std;

ros::Publisher ticks_publisher;

struct configuration{
    int translational_axis;
    int rotational_axis;
    int boost_button;
    int stop_button;
    int translational_multiplier;
    int rotational_multiplier;
    int boost_multiplier;
    string joy_topic;
    string ticks_topic;
    int debug;
};

struct configuration c;
uint32_t tick_sequence;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(!ros::ok()) return;
    //get absolute speed values, expressed in tick per interval
    int translational_velocity = joy->axes[c.translational_axis]*(float)c.translational_multiplier;
    int rotational_velocity    = joy->axes[c.rotational_axis]*(float)c.rotational_multiplier;
    //check if boost button is pressed
    if(joy->buttons[c.boost_button]==1){
        translational_velocity*=c.boost_multiplier;
        rotational_velocity*=c.boost_multiplier;
    }
    //now check if the stop button is pressed to halt the robot
    if(joy->buttons[c.stop_button]==1){
        translational_velocity=0;
        rotational_velocity=0;
    }
    capybarauno::capybara_ticks ct;
    ct.leftEncoder=-translational_velocity+rotational_velocity;
    ct.rightEncoder=translational_velocity+rotational_velocity;
    ct.header.stamp=ros::Time::now();
    ct.header.seq=tick_sequence;
    tick_sequence++;
    ticks_publisher.publish(ct);

}


void EchoParameters(){
    printf("%s %d\n","_translational_axis ",c.translational_axis);
    printf("%s %d\n","_rotational_axis ",c.rotational_axis);
    printf("%s %d\n","_boost_button ",c.boost_button);
    printf("%s %d\n","_stop_button ",c.stop_button);
    printf("%s %d\n","_translational_multiplier ",c.translational_multiplier);
    printf("%s %d\n","_rotational_multiplier ",c.rotational_multiplier);
    printf("%s %d\n","_boost_multiplier ",c.boost_multiplier);
    printf("%s %s\n","_joy_topic",c.joy_topic.c_str());
    printf("%s %s\n","_ticks_topic",c.ticks_topic.c_str());
    printf("%s %d\n","_debug",c.debug);
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "capybarauno_joy2ticks",ros::init_options::AnonymousName);
    ros::NodeHandle n("~");
    n.param("translational_axis", c.translational_axis, 1);
    n.param("rotational_axis", c.rotational_axis, 2);
    n.param("boost_button", c.boost_button, 5);
    n.param("stop_button", c.stop_button, 4);
    n.param("translational_multiplier", c.translational_multiplier, 10);
    n.param("rotational_multiplier", c.rotational_multiplier, 5);
    n.param("boost_multiplier", c.boost_multiplier, 2);
    n.param<string>("joy_topic", c.joy_topic, "/joy");
    n.param<string>("tick_topic", c.ticks_topic, "/requested_ticks");
    n.param("debug", c.debug, 1);

    EchoParameters();


    ros::Subscriber joypad_subscriber = n.subscribe(c.joy_topic.c_str(), 1000, joyCallback);
    ticks_publisher = n.advertise<capybarauno::capybara_ticks>(c.ticks_topic.c_str(), 1000);
    ros::spin();
    return 0;
}



