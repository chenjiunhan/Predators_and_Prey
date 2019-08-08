# Evolutionary Predator-prey Robot Systems: from simulationto real world
# Part I

This is a project about evolutionary predator-prey robot systems: from simulationto real world. The red predators are controlled by simple neural network, and the parameters of neural network is evolved by evolutionary algorithm, and the green prey is controlled by Gaussian gradient-based strategy.

The simulation results in Gazebo as shown in following video on Youtube:
<p align="center">
    <a href="http://www.youtube.com/watch?v=trR2Gc1tLzg"><img src="http://img.youtube.com/vi/trR2Gc1tLzg/0.jpg"></a>
</p>
<p> An result of evolution process in real world of robots arena as following video:</p>
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
4. Install Bayesian Optimization module for python3 https://github.com/fmfn/BayesianOptimization
   pip3 install bayesian-optimization
5. Install pip install Evolution Strategy module for python3 https://github.com/alirezamika/evostra
   pip3 install evostra
6. Install pip install CMA-ES module for python3 https://github.com/CMA-ES/pycma
   pip3 install cma
7. run script ./trainer.sh

# Part II

In part two, we integrated openAI Gym, Gazebo, ROS, and Robobo to create an envolutionary system. The system relies on sensor-based robots. The controllers of the predators and prey were evolved by the coevotionary framework with NEAT(NeuroEvolution of Augmenting Topologies).

Transfering the evolved robots from the simulation to the real world:
<p align="center">
    <a href="https://www.youtube.com/watch?v=GwM3_T5b3Lo"><img src="http://img.youtube.com/vi/GwM3_T5b3Lo/0.jpg"></a>
</p>

# Usage of simulation environment for Ubuntu 16.04

# Requirement
1. OpenAI Gym
2. ROS
3. Gazebo
4. NEAT(https://neat-python.readthedocs.io/en/latest/)

# Installation
1. copy all files in part2 to PATH/catkin/src
2. $ cd PATH/catkin
3. $ catkin_make install
4. $ cd PATH/catkin/devel
5. $ bash setup.bash

# Run
1. $ roslaunch robobo_gazebo robobo_world.launch 
2. open another terminal
3. $ roscd PATH/catkin_ws/src/predator_prey/robobo_gazebo/scripts
4. $ python3 robobo_coevolution_hall_of_fame_without_sensor.py
