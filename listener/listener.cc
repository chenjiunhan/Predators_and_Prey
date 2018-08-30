#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
  // Dump the message contents to stdout.
  //std::cout << _msg->DebugString();

  for (int i =0; i < posesStamped->pose_size(); ++i)
  {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
    
      const ::gazebo::msgs::Vector3d &position = pose.position();

      double x = position.x();
      double y = position.y();
      double z = position.z();
      std::cout << "Name: " << name; 
      std::cout << "Read position: x: " << x
          << " y: " << y << " z: " << z << std::endl;
  }
}
/////////////////////////////////////////////////

int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", posesStampedCallback);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);
    //std::cout << "?????????????";

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
