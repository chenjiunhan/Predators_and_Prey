#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

// listener
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <string>
#include <vector>

namespace gazebo {

    int world_reset = 0;
    int random_position = 1;   
    double prey_left_wheel_speed = 0.0;
    double prey_right_wheel_speed = 0.0;

    void preyLeftGzStringMsgCallback(const std::string & gzString) {
        
        double leftWheelSpeed = 0.0;

        //std::cout << gzString << "left";

        std::string s = gzString;
        std::string s2 = s.substr(2,8);
        //std::cout << s2 << "," << s2.length() << "XD2";
        leftWheelSpeed = std::stod(s2);

        prey_left_wheel_speed = leftWheelSpeed;
    }

    void preyRightGzStringMsgCallback(const std::string & gzString) {
        
        double rightWheelSpeed = 0.0;

        //std::cout << gzString << "right";

        std::string s = gzString;
        std::string s2 = s.substr(2,8);
        //std::cout << s2 << "," << s2.length() << "XD2";
        rightWheelSpeed = std::stod(s2);

        prey_right_wheel_speed = rightWheelSpeed;    
    }

    void worldResetGzStringMsgCallback(const std::string & gzString) {

        std::string s = gzString;        
        if (s.find("RESET") != std::string::npos) {
            world_reset = 1;   
        } else {
            world_reset = 0;   
            random_position = 1;
        }
    }

    class ModelPrey: public ModelPlugin {

        private: 
            transport::SubscriberPtr leftWheelSubscriber;
            transport::SubscriberPtr rightWheelSubscriber;            
            transport::SubscriberPtr worldReset;

        public:
            int predator_id = -1;
            int count_reset = 0;

        public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/ ) {

            // Store the pointer to the model
            this -> model = _model;
            
            gazebo::transport::NodePtr node1(new gazebo::transport::Node());
            node1 -> Init();

            gazebo::transport::NodePtr node2(new gazebo::transport::Node());
            node2 -> Init();

            // Create our node for communication
            //gazebo::transport::NodePtr node(new gazebo::transport::Node());
            //node->Init();

            // Listen to Gazebo world_stats topic
            //this->sub = node->Subscribe("~/pose/info", posesStampedCallback);

            // Listen to Gazebo world_stats topic
            std::string model_name = this -> model -> GetName();
            
            this -> leftWheelSubscriber = node1 -> Subscribe("~/" + model_name + "_left_wheel_speed", preyLeftGzStringMsgCallback);
            this -> rightWheelSubscriber = node2 -> Subscribe("~/" + model_name + "_right_wheel_speed", preyRightGzStringMsgCallback);
            gazebo::transport::NodePtr node3(new gazebo::transport::Node());
            node3 -> Init();

            this -> worldReset = node3 -> Subscribe("~/world_reset", worldResetGzStringMsgCallback);

            /*
            std::regex e ("(\\s+)(\\d+)");
            std::cmatch cm;

            if (std::regex_match (model_name, cm, e, std::regex_constants::match_default)) {
                this -> predator_id = std::stoi(cm[2]);                
            }
            */
            
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this -> updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind( & ModelPrey::OnUpdate, this));

        }

        // Called by the world update start event
        public: void OnUpdate() {
            // Apply a small linear velocity to the model.
            if (world_reset == 1) {
                //std::cout << "RRRRRRRRRRRRRRRR" << std::endl;
                double rand_x = ((rand() % 101) - 50) / 100.0;
                double rand_y = ((rand() % 101) - 50) / 100.0;


                ignition::math::Pose3d pose_prey = ignition::math::Pose3d(ignition::math::Vector3d(rand_x, rand_y, 0),
                                                                                  ignition::math::Quaterniond(0, 0, 0));
                                                                                       if (this->count_reset == 0) {
                    this->model->Reset();
                    //this -> model -> SetRelativePose(pose_prey);
                    this->count_reset += 1;
                }
                return;
            }

            this->count_reset = 0;

            std::string model_name = this -> model -> GetName();
            if (model_name.find("prey") != std::string::npos) {
                //std::cout << this->model->GetJointCount();
                //std::vector<gazebo::physics::JointPtr> jp = this->model->GetJoints();
                //std::cout << jp[0]->GetName("thymio_predator::left_wheel_hinge");
                //std::cout << std::to_string(prey_left_wheel_speed) << "," << std::to_string(prey_right_wheel_speed) << std::endl;
                this -> model -> GetJoint("thymio_prey::left_wheel_hinge") -> SetVelocity(0, prey_left_wheel_speed);
                this -> model -> GetJoint("thymio_prey::right_wheel_hinge") -> SetVelocity(0, prey_right_wheel_speed);
            }


        }

        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPrey)
}
