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
int beatcnt=0;
int beatingheart=0;
struct Packet_Decoder packet_decoder;
struct Packet packet;
struct Speed_Payload speedPayload;
struct State_Payload statePayload;
//heartbeat stuff
struct Packet heartbeat_packet;
struct Heartbeat_Payload heartbeat;

ros::Publisher ticks_publisher;

struct configuration{
    string serial_device;
    string published_ticks_topic;
    string subscribed_ticks_topic;
    int ascii;
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
    char* pEnd=Packet_write(&packet,buf,c.ascii);
    //send it
    sendToUart(serialFd,buf,pEnd-buf,0);
    if(c.debug){
        printf("SENDING: %s\n",buf);
    }
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
    }
}

void send_heartbeat(int &cnt){
    if(cnt%50==0){
        heartbeat_packet.seq++;
        char heartbuff[255];
        char* pEnd=Packet_write(&heartbeat_packet,heartbuff,c.ascii);
        sendToUart(serialFd,heartbuff,pEnd-heartbuff,0);
        const char heart_1[] = "\xe2\x99\xa1";
        const char heart_2[] = "\xe2\x99\xa5";
        if(beatingheart){
            printf("%s\n",heart_1);
            beatingheart=0;
        }else{
            printf("%s\n",heart_2);
            beatingheart=1;
        }

    }
    cnt++;
}

void EchoParameters(){
    printf("%s %s\n","_published_ticks_topic",c.published_ticks_topic.c_str());
    printf("%s %s\n","_subscribed_ticks_topic",c.subscribed_ticks_topic.c_str());
    printf("%s %s\n","_serial_device",c.serial_device.c_str());
    printf("%s %d\n","_ascii",c.ascii);
    printf("%s %d\n","_debug",c.debug);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capybarauno_node",ros::init_options::AnonymousName);
    ros::NodeHandle n("~");
    n.param<string>("serial_device", c.serial_device, "/dev/ttyACM0");
    n.param<string>("published_ticks_topic", c.published_ticks_topic, "/robot_ticks");
    n.param<string>("subscribed_ticks_topic", c.subscribed_ticks_topic, "/requested_ticks");
    n.param<int>("ascii", c.ascii, 1);
    n.param("debug", c.debug, 1);

    EchoParameters();
    RobotCommunication_init();

    heartbeat.beat=1;
    heartbeat_packet.id=Heartbeat_Payload_ID;
    heartbeat_packet.seq=0;
    heartbeat_packet.heartbeat=heartbeat;

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
        send_heartbeat(beatcnt);
        ros::spinOnce();
        usleep(1000);
    }


    return 0;
}



