#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

// listener, publisher
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/math/gzmath.hh>
#include <iostream>
#include <string>
#include <math.h>

// socket client
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>

#include <regex>
#include <unistd.h>
#include <mutex>

#define NUM_PREDATOR 4
#define NUM_PREY 1
#define NUM_WALL 4
#define PI 3.1415926

double p1_x = 0.0;
double p1_y = 0.0;
double leftWheelSpeed = 0.0;
double rightWheelSpeed = 0.0;

std::string *predator_positions = new std::string[NUM_PREDATOR]; // include orientation
std::string prey_position;
std::string *wall_positions = new std::string[NUM_WALL];

std::mutex mtx;

gazebo::common::Time sim_time;

static void toEulerAngle(const gazebo::msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	yaw = atan2(siny, cosy);
}

void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
    //std::cout << posesStamped->DebugString();
    std::regex regex_predator ("(predator)([0-9]+)");
    std::regex regex_wall ("(wall)([0-9]+)");

    std::smatch sm_predator;
    std::smatch sm_wall;
    
    std::string object_type;        

    for (int i =0; i < posesStamped->pose_size(); ++i)
    {
        const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
        std::string name = pose.name();
        std::regex_match (name, sm_predator, regex_predator);        
        std::regex_match (name, sm_wall, regex_wall);        

        object_type = "none";        

        //std::cout << "object_type: " << name << "," << object_type << std::endl;

        if (name == "prey") {
            object_type = "prey";
        } else if(sm_predator.size() > 1) {
            object_type = "predator";
        } else if(sm_wall.size() > 1){
            object_type = "wall";
        } else {
            //std::cout << "None!!!!!!!!!!!" << std::endl;
            continue;
        }

        const ::gazebo::msgs::Vector3d &position = pose.position();
        const ::gazebo::msgs::Quaternion &orientation = pose.orientation();

        double x = position.x();
        double y = position.y();
        double z = position.z();
        
        //double p1_x = x;
        //double p1_y = y;
        
        double roll;
        double pitch;
        double yaw;

        toEulerAngle(orientation, roll, pitch, yaw);
        
        std::string object_pos_info = std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(yaw);

        //std::cout << object_pos_info << std::endl;

        if (object_type == "prey") {
            prey_position = object_pos_info;
        }
        else if (object_type == "predator") {
            predator_positions[std::stoi(sm_predator[2]) - 1] = object_pos_info;
        }
        else if  (object_type == "wall") {                     
            wall_positions[std::stoi(sm_wall[2]) - 1] = object_pos_info;
        }
        //std::cout << "Name: " << name << std::endl; 
        //std::cout << "Read position: x: " << x << " y: " << y << " z: " << z << std::endl;
        //std::cout << std::abs(yaw - PI / 2) << std::endl;
        //std::cout << roll << ", " << pitch << ", " << yaw << std::endl << std::endl;

        /*if(std::abs(yaw - PI / 2) > 0.1) {
            leftWheelSpeed = 0.1;
            rightWheelSpeed = -0.1;
            //std::cout << "rotate";
        } else {
            leftWheelSpeed = 0.1;
            rightWheelSpeed = 0.1;
            //std::cout << "straight";

        }*/

        //std::cout << "Read position: x: " << x
        //    << " y: " << y << " z: " << z << std::endl;
    }
}

void cb_world_stats(ConstWorldStatisticsPtr &_msg)
{
    // Dump the message contents to stdout.
    const gazebo::msgs::Time sim_time_msg = _msg->sim_time();
    sim_time = (gazebo::common::Time) (gazebo::msgs::Convert(sim_time_msg));
}

int main(int _argc, char **_argv)
{
    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node_listener(new gazebo::transport::Node());
    node_listener->Init();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node_listener->Subscribe("~/pose/info", posesStampedCallback);

    // Create our node for communication
    gazebo::transport::NodePtr node_world_stats(new gazebo::transport::Node());
    node_world_stats->Init();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub_world_stats = node_world_stats->Subscribe("~/world_stats", cb_world_stats);

    gazebo::transport::NodePtr node_publisher(new gazebo::transport::Node());
    node_publisher->Init();

    // Publish to a Gazebo topic
    gazebo::transport::PublisherPtr *publisher_ptr_left = new gazebo::transport::PublisherPtr[NUM_PREDATOR];
    gazebo::transport::PublisherPtr *publisher_ptr_right = new gazebo::transport::PublisherPtr[NUM_PREDATOR];
    
    gazebo::transport::PublisherPtr prey_publisher_ptr_left;
    gazebo::transport::PublisherPtr prey_publisher_ptr_right;

    for(int i = 0; i < NUM_PREDATOR; i++) {
        publisher_ptr_left[i] = node_publisher->Advertise<gazebo::msgs::GzString>("~/predator" + std::to_string(i + 1) + "_left_wheel_speed");
        publisher_ptr_right[i] = node_publisher->Advertise<gazebo::msgs::GzString>("~/predator" + std::to_string(i + 1) + "_right_wheel_speed");

    }
    
    prey_publisher_ptr_left = node_publisher->Advertise<gazebo::msgs::GzString>("~/prey_left_wheel_speed");
    prey_publisher_ptr_right = node_publisher->Advertise<gazebo::msgs::GzString>("~/prey_right_wheel_speed");

    gazebo::transport::PublisherPtr publisher_world_reset = node_publisher->Advertise<gazebo::msgs::GzString>("~/world_reset");

    // Busy wait loop...replace with your own code as needed.

    double leftWheelSpeed = 0.0;
    double rightWheelSpeed = 0.0;
    
    gazebo::msgs::GzString msg1;
    gazebo::msgs::GzString msg2;
    gazebo::msgs::GzString msg3;
 
    int CreateSocket = 0,n = 0;
    char dataReceived[1024];
    struct sockaddr_in ipOfServer;
 
    memset(dataReceived, '0' ,sizeof(dataReceived));
 
    if((CreateSocket = socket(AF_INET, SOCK_STREAM, 0))< 0)
    {
        printf("Socket not created \n");
        return 1;
    }
 
    ipOfServer.sin_family = AF_INET;
    ipOfServer.sin_port = htons(9527);
    ipOfServer.sin_addr.s_addr = inet_addr("127.0.0.1");
 
    if(connect(CreateSocket, (struct sockaddr *)&ipOfServer, sizeof(ipOfServer))<0)
    {
        printf("Connection failed due to port and ip problems\n");
        return 1;
    }

    while(1)
    {
        gazebo::common::Time::MSleep(10);
        std::string gazebo_info = "";
        // TODO mutex

        gazebo_info += "p," + std::to_string(1) + "," + prey_position + ";";

        for(int i = 0; i < NUM_PREDATOR; i++) {
            gazebo_info += "P," + std::to_string(i+1) + "," + predator_positions[i] + ";";
        }

        for(int i = 0; i < NUM_WALL; i++) {
            gazebo_info += "w," + std::to_string(i+1) + "," + wall_positions[i] + ";";
        }

        gazebo_info += "t," + std::to_string(sim_time.Double()) + ";";


        send(CreateSocket, gazebo_info.c_str(), gazebo_info.length(),0);
        //std::cout << "Send: " << gazebo_info;
        n = recv(CreateSocket, dataReceived, sizeof(dataReceived)-1, 0);
        dataReceived[n] = 0;
        //std::cout << "Received: " << dataReceived << "," << n << std::endl;
        
        std::string received_string(dataReceived, n-1);
        if (received_string.length() > 4) {
            std::string reset_string = received_string.substr(0, 5);
            
            if (reset_string == "RESET") {
                std::string msg3_string = "RESET";
                //std::string msg3_string = "1";
                msg3.set_data(msg3_string);                
                //std::cout << predator_id << "," <<  leftWheelSpeed << "," << rightWheelSpeed << std::endl;
                publisher_world_reset->Publish(msg3);
                //std::cout << "RESET" << std::endl;
                std::string tmp_string = received_string.substr(5, sizeof(received_string) - 5);
                received_string = tmp_string;
            } else {
                std::string msg3_string = "NO";
                msg3.set_data(msg3_string);                
                //std::cout << predator_id << "," <<  leftWheelSpeed << "," << rightWheelSpeed << std::endl;
                publisher_world_reset->Publish(msg3);
            }
        } else {
            continue;
        }                


        std::string basic_regex = "(.+);";
        std::string whole_regex_string = "";
        for (int i = 0; i < NUM_PREDATOR + NUM_PREY; i++) {
            whole_regex_string += basic_regex;
        }
        std::regex whole_regex (whole_regex_string);
        std::smatch sm;
        std::regex_match (received_string, sm, whole_regex);
        for (int i=1; i < sm.size(); i++) {
            std::string delimiter = ",";
            size_t pos = 0;
            std::string token;
            int count_token = 0;
            int predator_id = -1;            
            std::string s = sm[i];
            //std::cout << s << std::endl;

            while ((pos = s.find(delimiter)) != std::string::npos) {
                token = s.substr(0, pos);
                //std::cout << i << "," << token << std::endl;
                s.erase(0, pos + delimiter.length());

                if(count_token == 0) {                    
                    if(i != sm.size() - 1) {
                        predator_id = std::stoi(token);
                    } 
                }
                if(count_token == 1) {
                    leftWheelSpeed = std::stod(token);
                }
                count_token++;
            }
            rightWheelSpeed = std::stod(s);

            //n = read(CreateSocket, dataReceived, sizeof(dataReceived)-1) > 0;
            //dataReceived[n] = 0;
            //std::cout << dataReceived << "," << n;

            //if(fputs(dataReceived, stdout) == EOF)
            //{
            //    printf("\nStandard output error");
            //}
            //printf("\n");

            //gazebo::common::Time::MSleep(10);        
            
            /*for(int i = 0; i < NUM_PREDATOR; i++) {
                
                switch(i) {
                    case 0:
                        leftWheelSpeed = 0.0;
                        rightWheelSpeed = 1.0;
                        break;                    
                    case 1:
                        leftWheelSpeed = 1.0;
                        rightWheelSpeed = 0.0;
                        break;
                    case 2:
                        leftWheelSpeed = 1.0;
                        rightWheelSpeed = 1.0;
                        break;
                }*/
            if(i != sm.size() - 1) {
                msg1.set_data(std::to_string(predator_id) + "," + std::to_string(leftWheelSpeed));
                msg2.set_data(std::to_string(predator_id) + "," + std::to_string(rightWheelSpeed));
                publisher_ptr_left[predator_id - 1]->Publish(msg1);
                publisher_ptr_right[predator_id - 1]->Publish(msg2);
            } else {
                msg1.set_data(std::to_string(leftWheelSpeed));
                msg2.set_data(std::to_string(rightWheelSpeed));
                prey_publisher_ptr_left->Publish(msg1);
                prey_publisher_ptr_right->Publish(msg2);

            }
            //std::cout << predator_id << "," <<  leftWheelSpeed << "," << rightWheelSpeed << std::endl;

            
        }
        //}
    }
 
    if( n < 0)
    {
        printf("Standard input error \n");
    }

    /*delete publisher_ptr_left;
    delete publisher_ptr_right;
    delete vector2dPtrLeft;
    delete vector2dPtrRight;*/

    // Make sure to shut everything down.
    gazebo::client::shutdown();

    delete predator_positions;
    delete wall_positions;
}


