//system stuff
#include <string>
//ros stuff
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "capybarauno/capybara_ticks.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
//own stuff
#include "malComm/mal_comm.h"
#include "malComm/mal_primitives.h"

using namespace std;

int serialFd;
int beatcnt=0;
int beatingheart=0;
struct Packet_Decoder packet_decoder;
struct Packet packet;
struct Speed_Payload speedPayload;
struct State_Payload statePayload;
struct Packet heartbeat_packet;
struct Heartbeat_Payload heartbeat;

ros::Publisher ticks_publisher;
ros::Publisher joint_ticks_publisher;
ros::Publisher odom_pub;

capybarauno::capybara_ticks currentTicks;
capybarauno::capybara_ticks previousTicks;

unsigned int _oldLeft=0;
unsigned int _oldRight=0;
double _oldJointLeft=0;
double _oldJointRight=0;
struct configuration{
    string serial_device;
    string published_ticks_topic;
    string published_joint_ticks_topic;
    string cmdvel_topic;
    string subscribed_ticks_topic;
    double kbaseline;
    double kleft;
    double kright;
    string published_odometry_topic;
    string published_link_name;
    string published_odom_link_name;
    int ascii;
    int debug;
    int heartbeat;

};

struct configuration c;
double x,y,t=0;
uint32_t tick_sequence=0;
int firstMessage=1;

void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    if(!ros::ok()) return;
    //get absolute speed values, expressed in tick per interval
    double translational_velocity = twist->linear.x;
    double rotational_velocity    = -twist->angular.z/(c.kbaseline*2);
    capybarauno::capybara_ticks ct;
    ct.leftEncoder=(uint16_t)((-translational_velocity+rotational_velocity)/(c.kleft*1000));
    ct.rightEncoder=(uint16_t)((translational_velocity+rotational_velocity)/(c.kright*1000));
    //ROS_INFO("CMDVEL %f %f %d %d",twist->linear.x,twist->angular.z,ct.leftEncoder,ct.rightEncoder);
    speedPayload.leftTick=ct.leftEncoder;
    speedPayload.rightTick=ct.rightEncoder;
    packet.speed=speedPayload;
    char buf[255];
    char* pEnd=Packet_write(&packet,buf,c.ascii);
    sendToUart(serialFd,buf,pEnd-buf,0);

}


Packet read_ticks_from_uart(int serialDevice, Packet_Decoder& decoder){
    char chunk;
    int complete=0;
    Packet read_packet;
    while(read(serialDevice, &chunk, 1)>0 && !complete){
        complete = Packet_Decoder_putChar(&decoder,(unsigned char)chunk);
    }
    if(complete){
        Packet_parse(decoder.buffer_start,&read_packet,c.ascii);
    }
    return read_packet;
}

void RobotCommunication_init(){
    initConsts();
    Packet_Decoder_init(&packet_decoder,c.ascii);
    packet.id=Speed_Payload_ID;
    serialFd=openPort((char*)c.serial_device.c_str());
    if(c.debug){
        printf("serial port status: %d\n",serialFd);
        fflush(stdout);
    }
}

void send_heartbeat(int &cnt){
    if(cnt%2==0){
        heartbeat_packet.seq++;
        char heartbuff[255];
        char* pEnd=Packet_write(&heartbeat_packet,heartbuff,c.ascii);
        sendToUart(serialFd,heartbuff,pEnd-heartbuff,0);
//        if(c.debug){
//            printf("SENDING: %s\n",heartbuff);
//            fflush(stdout);
//        }

    }
    cnt++;
}

void sendOdometry(tf::TransformBroadcaster& odom_broadcaster, capybarauno::capybara_ticks ticks){
    uint16_t ult;
    uint16_t urt;
    if(firstMessage){
        previousTicks.leftEncoder=ticks.leftEncoder;
        previousTicks.rightEncoder=ticks.rightEncoder;
        firstMessage=0;
    }
    else{
    ult = ticks.leftEncoder-previousTicks.leftEncoder;
    urt = ticks.rightEncoder-previousTicks.rightEncoder;
    previousTicks.leftEncoder=ticks.leftEncoder;
    previousTicks.rightEncoder=ticks.rightEncoder;
    int16_t lt = (int16_t)ult;
    int16_t rt = (int16_t)urt;


    double left=-(double)lt*c.kleft;
    double right=(double)rt*c.kright;
    double s = (left+right)/2;
    t-=(right-left)/c.kbaseline;
    x+=s*cos(t);
    y+=s*sin(t);
    ROS_INFO("ODOM: %f",t);

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

}

void EchoParameters(){
    printf("%s %s\n","_published_ticks_topic",c.published_ticks_topic.c_str());
    printf("%s %s\n","_published_joint_ticks_topic",c.published_joint_ticks_topic.c_str());
    printf("%s %s\n","_subscribed_ticks_topic",c.subscribed_ticks_topic.c_str());
    printf("%s %s\n","_serial_device",c.serial_device.c_str());
    printf("%s %s\n","_published_odometry_topic",c.published_odometry_topic.c_str());
    printf("%s %s\n","_published_link_name",c.published_link_name.c_str());
    printf("%s %s\n","_published_odom_link_name",c.published_odom_link_name.c_str());
    printf("%s %f\n","_kleft",c.kleft);
    printf("%s %f\n","_kright",c.kright);
    printf("%s %f\n","_baseline",c.kbaseline);
    printf("%s %d\n","_ascii",c.ascii);
    printf("%s %d\n","_debug",c.debug);
    printf("%s %d\n","_hearbeat",c.heartbeat);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capybarauno_node",ros::init_options::AnonymousName);
    tf::TransformBroadcaster odom_broadcaster;
    ros::NodeHandle n("~");
    n.param<string>("serial_device", c.serial_device, "/dev/ttyACM0");
    n.param<string>("published_ticks_topic", c.published_ticks_topic, "/robot_ticks");
    n.param<string>("published_joint_ticks_topic", c.published_joint_ticks_topic, "/joint_robot_ticks");
    n.param<string>("cmdvel_topic", c.cmdvel_topic, "/cmd_vel");
    n.param<string>("published_odometry_topic", c.published_odometry_topic, "/odom");
    n.param<string>("published_link_name", c.published_link_name, "/base_link");
    n.param<string>("published_odom_link_name", c.published_odom_link_name, "/odom");
    n.param<double>("kleft", c.kleft, 0.0058);
    n.param<double>("kright", c.kright, 0.0058);
    n.param<double>("kbaseline", c.kbaseline, 0.2);
    n.param<int>("ascii", c.ascii, 1);
    n.param<int>("heartbeat", c.heartbeat, 1);
    n.param("debug", c.debug, 1);

    EchoParameters();
    RobotCommunication_init();

    heartbeat.beat=1;
    heartbeat_packet.id=Heartbeat_Payload_ID;
    heartbeat_packet.seq=0;
    heartbeat_packet.heartbeat=heartbeat;

    previousTicks.leftEncoder=0;
    previousTicks.rightEncoder=0;
    currentTicks.leftEncoder=0;
    currentTicks.rightEncoder=0;
    ros::Subscriber cmdvel_subscriber = n.subscribe(c.cmdvel_topic.c_str(), 1000, cmdvelCallback);
    ticks_publisher = n.advertise<capybarauno::capybara_ticks>(c.published_ticks_topic.c_str(), 1000);
    joint_ticks_publisher = n.advertise<sensor_msgs::JointState>(c.published_joint_ticks_topic.c_str(), 1000);
    odom_pub = n.advertise<nav_msgs::Odometry>(c.published_odometry_topic.c_str(), 1000);
    //ros::ok() used to get the SIGINT ctrl+c
    while(ros::ok()){
        Packet robot_data = read_ticks_from_uart(serialFd,packet_decoder);
        capybarauno::capybara_ticks currentTicks;
        sensor_msgs::JointState currentJointTicks;
        currentTicks.header.seq = robot_data.seq;
        currentTicks.header.stamp = ros::Time::now();
        currentTicks.leftEncoder = robot_data.state.leftEncoder;
        currentTicks.rightEncoder = robot_data.state.rightEncoder;
        currentJointTicks.name.resize(4);
        currentJointTicks.position.resize(4);
        currentJointTicks.name[0]="left signed";
        currentJointTicks.name[1]="right signed";
        currentJointTicks.name[2]="left";
        currentJointTicks.name[3]="right";
        currentJointTicks.position[0]=(double)(-(signed int)((_oldLeft-robot_data.state.leftEncoder)));
        currentJointTicks.position[1]=(double)(signed int)(_oldRight-robot_data.state.rightEncoder);
        currentJointTicks.position[2]=_oldJointLeft+(double)robot_data.state.leftEncoder;
        currentJointTicks.position[3]=_oldJointRight+(double)robot_data.state.rightEncoder;
        _oldJointLeft = currentJointTicks.position[2];
        _oldJointRight = currentJointTicks.position[3];
        _oldLeft = robot_data.state.leftEncoder;
        _oldRight = robot_data.state.rightEncoder;
        joint_ticks_publisher.publish(currentJointTicks);
        ticks_publisher.publish(currentTicks);
        sendOdometry(odom_broadcaster,currentTicks);
        if(c.heartbeat==1){
            send_heartbeat(beatcnt);
        }
        ros::spinOnce();
        usleep(1000);
    }


    return 0;
}

