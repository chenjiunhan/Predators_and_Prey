# Predators and prey
This is a project about predators and prey. The red predators are controller by simple neural network, and the parameters of neural network is evolved by evolutionary algorithm, and the green prey is controlled by Gaussian gradient-based controller.

The simulation is in Gazebo as following video on Youtube:
<p align="center">
    <a href="http://www.youtube.com/watch?v=trR2Gc1tLzg"><img src="http://img.youtube.com/vi/trR2Gc1tLzg/0.jpg"></a>
</p>
<p>The evolution process in real world:</p>
<br>
<p align="center">
    <a href="http://www.youtube.com/watch?v=fjTd06L-9bQ"><img src="http://img.youtube.com/vi/fjTd06L-9bQ/0.jpg"></a>
</p>
<p>This system can be used for numerous robots, at least it works on 8 predators in such small field:</p>
<br>
<p align="center">
    <a href="http://www.youtube.com/watch?v=Uhq7wDz3G_Q"><img src="http://img.youtube.com/vi/Uhq7wDz3G_Q/0.jpg"></a>
</p>

# Usage of simulation environment for Ubuntu 16.04
1. Install Gazebo 7 http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0
2. Copy models folder to ~/.gazebo/models
3. Add 
   export GAZEBO_PLUGIN_PATH=/"PATH TO PLUGIN"/plugin/build:$GAZEBO_PLUGIN_PATH
   to ~/.bashrc and execute command 
   source ~/.bashrc
4. run script ./trainer.sh
