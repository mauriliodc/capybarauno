//system stuff
#include <string>
//ros stuff
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "capybarauno/capybara_ticks.h"
//own stuff
#include "malComm/mal_comm.h"
#include "malComm/mal_primitives.h"

using namespace std;

int serialFd;
struct Packet_Decoder packet_decoder;
struct Packet packet;
struct Speed_Payload speedPayload;
struct State_Payload statePayload;
int ascii=1;

ros::Publisher ticks_publisher;

struct configuration{
    string serial_device;
    string published_ticks_topic;
    string subscribed_ticks_topic;
    int debug;

};

struct configuration c;

void ticksCallback(const capybarauno::capybara_ticksConstPtr& ticks)
{
    if(!ros::ok()) return;
    speedPayload.leftTick=ticks->leftEncoder;
    speedPayload.rightTick=ticks->rightEncoder;
    //assign the payload to the general packet
    packet.speed=speedPayload;
    char buf[255];
    char* pEnd=Packet_write(&packet,buf,ascii);
    //send it
    sendToUart(serialFd,buf,pEnd-buf,0);
    if(c.debug){
        printf("SENDING: %s\n",buf);
    }
}


Packet read_ticks_from_uart(int serialDevice, Packet_Decoder& decoder){
    char c;
    int complete=0;
    Packet read_packet;
    while(read(serialDevice, &c, 1)>0 && !complete){
        complete = Packet_Decoder_putChar(&decoder,(unsigned char)c);
    }
    if(complete){
        Packet_parse(decoder.buffer_start,&read_packet,ascii);
    }
    return read_packet;
}

void RobotCommunication_init(){
    initConsts();
    Packet_Decoder_init(&packet_decoder,ascii);
    packet.id=Speed_Payload_ID;
    serialFd=openPort((char*)c.serial_device.c_str());
    if(c.debug){
        printf("serial port status: %d\n",serialFd);
    }
}

void EchoParameters(){
    printf("%s %s\n","_published_ticks_topic",c.published_ticks_topic.c_str());
    printf("%s %s\n","_subscribed_ticks_topic",c.subscribed_ticks_topic.c_str());
    printf("%s %s\n","_serial_device",c.serial_device.c_str());
    printf("%s %d\n","_debug",c.debug);
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "capybarauno_node",ros::init_options::AnonymousName);
    ros::NodeHandle n("~");
    n.param<string>("serial_device", c.serial_device, "/dev/ttyACM0");
    n.param<string>("published_ticks_topic", c.published_ticks_topic, "/robot_ticks");
    n.param<string>("subscribed_ticks_topic", c.subscribed_ticks_topic, "/requested_ticks");
    n.param("debug", c.debug, 1);

    EchoParameters();
    RobotCommunication_init();

    ros::Subscriber ticks_subscriber = n.subscribe(c.subscribed_ticks_topic.c_str(), 1000, ticksCallback);
    ticks_publisher = n.advertise<capybarauno::capybara_ticks>(c.published_ticks_topic.c_str(), 1000);

    //ros::ok() used to get the SIGINT ctrl+c
    while(ros::ok()){
        Packet robot_data = read_ticks_from_uart(serialFd,packet_decoder);
        capybarauno::capybara_ticks ticks_message;
        ticks_message.header.seq = robot_data.seq;
        ticks_message.header.stamp = ros::Time::now();
        ticks_message.leftEncoder = robot_data.state.leftEncoder;
        ticks_message.rightEncoder = robot_data.state.rightEncoder;
        ticks_publisher.publish(ticks_message);

        ros::spinOnce();
        usleep(1000);
    }


    return 0;
}



