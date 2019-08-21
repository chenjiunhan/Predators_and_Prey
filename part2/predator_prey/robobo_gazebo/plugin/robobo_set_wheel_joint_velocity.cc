#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <string>
#include <vector>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include <thread>
#include "std_msgs/Int32MultiArray.h"

//#define NUM_PREDATOR 4

namespace gazebo {

    int left_wheel_speed_p = 0;
    int right_wheel_speed_p = 0;
    int left_wheel_speed_0 = 0;
    int right_wheel_speed_0 = 0;
    int left_wheel_speed_1 = 0;
    int right_wheel_speed_1 = 0;
    int left_wheel_speed_2 = 0;
    int right_wheel_speed_2 = 0;
    
    void leftVector2dMsgCallback(ConstVector2dPtr & vector2d) {
        //std::cout << vector2d -> DebugString();
        //std::cout << vector2d->y();
        //left_wheel_speed[i] = vector2d -> y();
    }

    void rightVector2dMsgCallback(ConstVector2dPtr & vector2d) {
        //std::cout << vector2d -> DebugString();
        //std::cout << vector2d->y();
        //right_wheel_speed[i] = vector2d -> y();
    }
    
    void leftGzStringMsgCallback(const std::string & gzString) {
        
        /*int predator_id = -1;
        double leftWheelSpeed = 0.0;

        //std::cout << gzString << "left";

        std::string delimiter = ",";
        size_t pos = 0;
        std::string token;
        int count_token = 0;

        std::string s = gzString;

        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            //std::cout << token << std::endl;
            s.erase(0, pos + delimiter.length());

            if(count_token == 0) {
                predator_id = std::stoi(token);
            }
            count_token++;            
        }
        leftWheelSpeed = std::stod(s);

        left_wheel_speed[predator_id - 1] = leftWheelSpeed;*/
    }

    void rightGzStringMsgCallback(const std::string & gzString) {
        /*int predator_id = -1;
        double rightWheelSpeed = 0.0;

        //std::cout << gzString << "right";

        std::string delimiter = ",";
        size_t pos = 0;
        std::string token;
        int count_token = 0;

        std::string s = gzString;

        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            //std::cout << token << "??????" << std::endl;
            s.erase(0, pos + delimiter.length());

            if(count_token == 0) {
                predator_id = std::stoi(token);
            }

            count_token++;            
        }
        rightWheelSpeed = std::stod(s);

        right_wheel_speed[predator_id - 1] = rightWheelSpeed;    */
    }  

    class RoboboPlugin : public ModelPlugin {

        private: 
            transport::SubscriberPtr wheelSubscriber;
            
        /// \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSub;

        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;

        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;

        public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/ ) {                                              
                                   
            std::cout << "STTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTART" << std::endl;            
                                   
            this -> model = _model;

            // Create our node for communication
            gazebo::transport::NodePtr node1(new gazebo::transport::Node());
            node1 -> Init();

            gazebo::transport::NodePtr node2(new gazebo::transport::Node());
            node2 -> Init();           
            
            //this -> leftWheelSubscriber = node1 -> Subscribe("~/" + model_name + "_left_wheel_speed", leftVector2dMsgCallback);
            //this -> rightWheelSubscriber = node2 -> Subscribe("~/" + model_name + "_right_wheel_speed", rightVector2dMsgCallback);
            //this -> wheelSubscriber = node1 -> Subscribe("/" + this -> model -> GetName() + "/wheel_vel_cmd", leftGzStringMsgCallback);
                       
            // simulation iteration.
            //this -> updateConnection = event::Events::ConnectWorldUpdateBegin(
             //   std::bind( & RoboboPlugin::OnUpdate, this));

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
              int argc = 0;
              char **argv = NULL;
              ros::init(argc, argv, "gazebo_client",
                  ros::init_options::NoSigintHandler);
            }
            
            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
              ros::SubscribeOptions::create<std_msgs::Int32MultiArray>(
                  "/" + this->model->GetName() + "/vel_cmd",
                  1,
                  boost::bind(&RoboboPlugin::OnRosMsg, this, _1),
                  ros::VoidPtr(), &this->rosQueue);
            this->rosSub = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
              std::thread(std::bind(&RoboboPlugin::QueueThread, this));


            this -> updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind( & RoboboPlugin::OnUpdate, this));
        }
        
        public: void SetVelocity(const std::vector<std::int32_t> &_vel)
        {
            //std::cout << this -> model -> GetName() << std::endl;
            //std::cout << _vel[0] << std::endl;
            //std::cout << _vel[1] << std::endl;
            this -> model -> GetJoint(this -> model -> GetName() + "::robobo_right_wheel_joint") -> SetVelocity(0, _vel[1]);
            this -> model -> GetJoint(this -> model -> GetName() + "::robobo_left_wheel_joint") -> SetVelocity(0, _vel[0]);
            
            
        }
        
        public: void OnUpdate() {                    
        
            //std::cout << "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ" << std::endl;     
            if(this -> model -> GetName().compare("prey") == 0) {
                //std::cout << left_wheel_speed_p <<  ", "<<  right_wheel_speed_p << std::endl;
                this -> model -> GetJoint(this -> model -> GetName() + "::robobo_left_wheel_joint") -> SetVelocity(0, left_wheel_speed_p);
                this -> model -> GetJoint(this -> model -> GetName() + "::robobo_right_wheel_joint") -> SetVelocity(0, right_wheel_speed_p);
            } else if(this -> model -> GetName().compare("predator0") == 0) {
                //std::cout << left_wheel_speed_p <<  ", "<<  right_wheel_speed_p << std::endl;
                this -> model -> GetJoint(this -> model -> GetName() + "::robobo_left_wheel_joint") -> SetVelocity(0, left_wheel_speed_0);
                this -> model -> GetJoint(this -> model -> GetName() + "::robobo_right_wheel_joint") -> SetVelocity(0, right_wheel_speed_0);
            } else if(this -> model -> GetName().compare("predator1") == 0) {
                //std::cout << left_wheel_speed_p <<  ", "<<  right_wheel_speed_p << std::endl;
                this -> model -> GetJoint(this -> model -> GetName() + "::robobo_left_wheel_joint") -> SetVelocity(0, left_wheel_speed_1);
                this -> model -> GetJoint(this -> model -> GetName() + "::robobo_right_wheel_joint") -> SetVelocity(0, right_wheel_speed_1);
            } else if(this -> model -> GetName().compare("predator2") == 0) {
                //std::cout << left_wheel_speed_p <<  ", "<<  right_wheel_speed_p << std::endl;
                this -> model -> GetJoint(this -> model -> GetName() + "::robobo_left_wheel_joint") -> SetVelocity(0, left_wheel_speed_2);
                this -> model -> GetJoint(this -> model -> GetName() + "::robobo_right_wheel_joint") -> SetVelocity(0, right_wheel_speed_2);
            }
        }

        public: void OnRosMsg(const std_msgs::Int32MultiArray::ConstPtr &_msg)
        {            
            //std::cout << "YYYYYYYYYYYYYYYYYYYYYYYY" << std::endl;     
            if(this -> model -> GetName().compare("prey") == 0) {
                left_wheel_speed_p = _msg->data[0];
                right_wheel_speed_p = _msg->data[1];
            } else if(this -> model -> GetName().compare("predator0") == 0) {
                left_wheel_speed_0 = _msg->data[0];
                right_wheel_speed_0 = _msg->data[1];
            } else if(this -> model -> GetName().compare("predator1") == 0) {
                left_wheel_speed_1 = _msg->data[0];
                right_wheel_speed_1 = _msg->data[1];
            } else if(this -> model -> GetName().compare("predator2") == 0) {
                left_wheel_speed_2 = _msg->data[0];
                right_wheel_speed_2 = _msg->data[1];
            }
            
            /* else {
                this->SetVelocity(_msg->data);
            }*/
            
        }

        /// \brief ROS helper function that processes messages
        private: void QueueThread()
        {
        
            //std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;            
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        // Called by the world update start event
        //public: void OnUpdate() {
                        
        //}

        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(RoboboPlugin)
}
