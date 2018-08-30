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

#define NUM_PREDATOR 4

namespace gazebo {

    double *left_wheel_speed = new double[NUM_PREDATOR];
    double *right_wheel_speed = new double[NUM_PREDATOR];        

    int *model_reset = new int[NUM_PREDATOR];
    int world_reset = 0;
    
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
        
        int predator_id = -1;
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

        left_wheel_speed[predator_id - 1] = leftWheelSpeed;
    }

    void rightGzStringMsgCallback(const std::string & gzString) {
        int predator_id = -1;
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

        right_wheel_speed[predator_id - 1] = rightWheelSpeed;    
    }

    void worldResetGzStringMsgCallback(const std::string & gzString) {

        std::string s = gzString;        
        if (s.find("RESET") != std::string::npos) {
            world_reset = 1;   
        } else {
            world_reset = 0;   
        }
    }

    class ModelPush: public ModelPlugin {

        private: 
            transport::SubscriberPtr leftWheelSubscriber;
            transport::SubscriberPtr rightWheelSubscriber;
            transport::SubscriberPtr worldReset;
            transport::SubscriberPtr sub;

        public:
            int predator_id = -1;

        public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/ ) {

            for(int i = 0; i < NUM_PREDATOR ; i++) {
                left_wheel_speed[i] = 0.0;               
                right_wheel_speed[i] = 0.0;
            }

            // Store the pointer to the model
            this -> model = _model;

            // Create our node for communication
            gazebo::transport::NodePtr node1(new gazebo::transport::Node());
            node1 -> Init();

            gazebo::transport::NodePtr node2(new gazebo::transport::Node());
            node2 -> Init();

            gazebo::transport::NodePtr node3(new gazebo::transport::Node());
            node3 -> Init();


            // Create our node for communication
            //gazebo::transport::NodePtr node(new gazebo::transport::Node());
            //node->Init();

            // Listen to Gazebo world_stats topic
            //this->sub = node->Subscribe("~/pose/info", posesStampedCallback);

            // Listen to Gazebo world_stats topic
            std::string model_name = this -> model -> GetName();
            
            //this -> leftWheelSubscriber = node1 -> Subscribe("~/" + model_name + "_left_wheel_speed", leftVector2dMsgCallback);
            //this -> rightWheelSubscriber = node2 -> Subscribe("~/" + model_name + "_right_wheel_speed", rightVector2dMsgCallback);
            this -> leftWheelSubscriber = node1 -> Subscribe("~/" + model_name + "_left_wheel_speed", leftGzStringMsgCallback);
            this -> rightWheelSubscriber = node2 -> Subscribe("~/" + model_name + "_right_wheel_speed", rightGzStringMsgCallback);
            
            this -> worldReset = node3 -> Subscribe("~/world_reset", worldResetGzStringMsgCallback);

            /*
            std::regex e ("(\\s+)(\\d+)");
            std::cmatch cm;

            if (std::regex_match (model_name, cm, e, std::regex_constants::match_default)) {
                this -> predator_id = std::stoi(cm[2]);                
            }
            */
            
            this -> predator_id = std::stoi(model_name.substr(8,1));
            
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this -> updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind( & ModelPush::OnUpdate, this));

        }

        // Called by the world update start event
        public: void OnUpdate() {
            // Apply a small linear velocity to the model.
            if (world_reset == 1) {
                //std::cout << "RRRRRRRRRRRRRRRR" << std::endl;
                this->model->Reset();
                return;
            }

            //std::string model_name = this -> model -> GetName();
            //if (model_name.find("predator") != std::string::npos) {
            //std::cout << this->model->GetName() << std::endl << predator_id << std::endl << left_wheel_speed[this -> predator_id - 1];

                //std::vector<gazebo::physics::JointPtr> jp = this->model->GetJoints();
                //std::cout << jp[0] ->GetName("thymio_predator::left_wheel_hinge");
                //std::cout << jp[0]->GetName("thymio_predator::left_wheel_hinge");
            this -> model -> GetJoint("thymio_predator::left_wheel_hinge") -> SetVelocity(0, left_wheel_speed[this -> predator_id - 1]);
            this -> model -> GetJoint("thymio_predator::right_wheel_hinge") -> SetVelocity(0, right_wheel_speed[this -> predator_id - 1]);
            //}
        }

        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
