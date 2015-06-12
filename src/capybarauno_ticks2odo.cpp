//system stuff
#include <string>
#include <stdint.h>
//ros stuff
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "capybarauno/capybara_ticks.h"
#include "capybarauno/capybara_ticks_signed.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"


using namespace std;

ros::Publisher odom_pub;
ros::Publisher ticks_publisher;
uint32_t ticks_publisherSeq=0;

capybarauno::capybara_ticks currentTicks;
capybarauno::capybara_ticks previousTicks;

int16_t leftSignedTicks;
int16_t rightSignedTicks;
int firstMessage=1;

struct configuration{
    string kbaseline;
    string kleft;
    string kright;
    string subscribed_ticks_topic;
    string published_ticks_topic;
    string published_odometry_topic;
    string published_link_name;
    string published_odom_link_name;
    int debug;
};

struct configuration c;
float x,y,t=0;
float kb,kr,kl;

void ticksCallback(const capybarauno::capybara_ticksConstPtr& ticks)
{
    if(!ros::ok()) return;

    if(firstMessage){
        currentTicks.leftEncoder=ticks->leftEncoder;
        currentTicks.rightEncoder=ticks->rightEncoder;
        previousTicks.leftEncoder=ticks->leftEncoder;
        previousTicks.rightEncoder=ticks->rightEncoder;
        firstMessage=0;
    }else{
        currentTicks.leftEncoder = ticks->leftEncoder - previousTicks.leftEncoder;
        currentTicks.rightEncoder = ticks->rightEncoder - previousTicks.rightEncoder;
        previousTicks.leftEncoder=ticks->leftEncoder;
        previousTicks.rightEncoder=ticks->rightEncoder;

        leftSignedTicks=-(int16_t)currentTicks.leftEncoder;
        rightSignedTicks=(int16_t)currentTicks.rightEncoder;

        float lt=(float)leftSignedTicks*kl;
        float rt=(float)rightSignedTicks*kr;
        if(c.debug){
            cerr << "lt: "<<lt<<" rt: "<<rt<<endl;
        }

        float s = (lt*kl+rt*kr)/2;
        t-=(rt*kr-lt*kl)/kb;
        if(c.debug){
            cerr << "\ttheta: "<<t<<endl;
        }
        x+=s*cos(t);
        y+=s*sin(t);
        capybarauno::capybara_ticks_signed ct;
        ct.leftEncoder=leftSignedTicks;
        ct.rightEncoder=rightSignedTicks;
        ct.header.stamp=ros::Time::now();
        ct.header.seq=ticks_publisherSeq;
        ticks_publisherSeq++;
        ticks_publisher.publish(ct);

    }

}

void sendOdometry(tf::TransformBroadcaster& odom_broadcaster){
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(t);
    geometry_msgs::TransformStamped odom_trans;
    ros::Time current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = c.published_odometry_topic.c_str();
    odom_trans.child_frame_id = c.published_link_name.c_str();
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = c.published_odom_link_name;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom_pub.publish(odom);

}


void EchoParameters(){
    printf("%s %s\n","_published_odometry_topic",c.published_odometry_topic.c_str());
    printf("%s %s\n","_published_link_name",c.published_link_name.c_str());
    printf("%s %s\n","_published_odom_link_name",c.published_odom_link_name.c_str());
    printf("%s %s\n","_subscribed_ticks_topic",c.subscribed_ticks_topic.c_str());
    printf("%s %s\n","_published_ticks_topic",c.published_ticks_topic.c_str());
    printf("%s %s\n","_kbaseline",c.kbaseline.c_str());
    printf("%s %s\n","_kleft",c.kleft.c_str());
    printf("%s %s\n","_kright",c.kright.c_str());
    printf("%s %d\n","_debug",c.debug);
}

int main(int argc, char **argv)
{

    currentTicks.leftEncoder=0;
    currentTicks.rightEncoder=0;
    previousTicks.leftEncoder=0;
    previousTicks.rightEncoder=0;
    leftSignedTicks=0;
    rightSignedTicks=0;

    ros::init(argc, argv, "capybarauno_ticks2odo",ros::init_options::AnonymousName);
    tf::TransformBroadcaster odom_broadcaster;
    ros::NodeHandle n("~");
    n.param<string>("kbaseline", c.kbaseline, "0.2f");
    n.param<string>("kleft", c.kleft, "0.001f");
    n.param<string>("kright", c.kright, "0.001f");

    n.param<string>("published_ticks_topic", c.published_ticks_topic, "/relative_signed_ticks");
    n.param<string>("published_odometry_topic", c.published_odometry_topic, "/odom");
    n.param<string>("published_link_name", c.published_link_name, "/base_link");
    n.param<string>("published_odom_link_name", c.published_odom_link_name, "/odom");
    n.param<string>("subscribed_ticks_topic", c.subscribed_ticks_topic, "/robot_ticks");
    n.param("debug", c.debug, 1);

    EchoParameters();

    kb = atof(c.kbaseline.c_str());
    kr = atof(c.kright.c_str());
    kl = atof(c.kleft.c_str());

    ros::Subscriber ticks_subscriber = n.subscribe(c.subscribed_ticks_topic.c_str(), 1000, ticksCallback);
    odom_pub = n.advertise<nav_msgs::Odometry>(c.published_odometry_topic.c_str(), 1000);
    ticks_publisher = n.advertise<capybarauno::capybara_ticks_signed>(c.published_ticks_topic.c_str(), 1000);
    while(ros::ok()){
        ros::spinOnce();
        sendOdometry(odom_broadcaster);
        usleep(1000);
    }

    return 0;
}
