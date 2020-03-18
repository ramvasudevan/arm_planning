// copied and pasted from Corina's tutorial:

# How to run Fetch controllers
## You will need an Ubuntu Computer for this step

1. Power on Fetch: press the round button on the side of the robot.


2. Connect either to:
a) The wireless network of the robot: Fetchy-Wireless with password drop1234
       - this will have some lag in terms of data communication
b) Use the wired connection: on your computer add a new Manual Ethernet Network Connection that is in the same network as the robot 192.168.1.X (!!!!! Do not choose 192.168.1.100 because that is the IP of the robot!!!!!)

3. Once you are connected to the same network you can check if there is communication with the robot by opening a terminal on your computer and running:
        ping 192.168.1.100

4. Next step is to ssh into the robot. Open a terminal and execute:
	ssh -X fetch@192.168.1.100
           password: robotics

5. If there will be other code running on a different machine than on the fetch computer then you have to set the network communications for ROS. To do this there are 2 steps:
a) On the fetch computer, in the terminal where you did the ssh and where you will run the controllers code:
            export ROS_IP=192.168.1.100
b) On your computer where you will run external code (!!! in the same network to fetch!!!!)
           export ROS_IP=192.168.1.X
           export ROS_MASTER_URI=http://192.168.1.100:11311

6. Perfect!!!! Now you should be able to run the controllers for fetch. So, in the terminal where you are on fetch computer and where you ran the export command execute
// Patrick's edit 2020/01/08: first cd catkin_ws, then source devel/setup.bash.
	roslaunch fetch_demos fetch_planning_interface.launch

7. Now the controllers are working and they expect input from the path planner. This has to be send through ROS MESSAGES, of Type: sensor_msgs/JointState (here is the documentation how this data has to be formated http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html). Also the Name of the topic has to be: "/fetch/des_states".
The process to start the controller expects either desired joint positions OR desired joint positions and desired joint velocities


8. To record data from the robot you could use rosbags. You should run on your computer on a new terminal (were previously did the step in 7.b)
    rosbag record -O custom_name_of_bag.bag /fetch/des_states /joint_states /topic_name_of_interest

9. When you are done, power the robot off by pressing the same button as in step 1, but keeping it pressed for a few seconds until you see the light around it blinking white


10. Debugging:
a) check if the robot state is available by
rostopic echo /joint_states
- if nothing is returned, power on and off the robot – this will fix the issue
- if this is not the case check all the emergency buttons and on/off buttons around the robot to not be pressed down or to 0 !!! Especially the one in the back of the robot

b) if the arm just drops
- the controllers are cut off because the input data for them is not appropriate
- check the planning sending commands to be sure there is some data publised (especially on positions)
- after fixing the bug, power off and on the robot and things should be back and running

c) if the arm just doesn’t move
- there is a problem with the communication between the robot and the external computer where the planning runns
- be sure the steps in point 5 are done properly!!! check the IP adresses
- to be sure the communication is up and running, execute the following command on your computer’s terminal:
rostopic echo /joint_states
- if there is no information received then the comms are not set correctly

d) !!!!! DO NOT LET THE BATTERY DIE !!!!
- keep the robot plugged in the charger almost all the time (powered OFF or powered ON when you don’t need the base moving around).
- the only time you shouldn't have it charging is when you want to run something you haven’t tried before and you are not convinced that is going to be a safe movement. By not having it plugged is going to ensure that the long wired emergency stop will work
- when charging the button is blinking green, when fully charged is constant green
- if the light is blinking red or solid red – either the batteries are almost dead or the small button in the back of the robot is off (0)

e) if the robot just dies – you pressed accidentally the button in the back of the robot

## Once all the above steps have been done...
You should be able to run the ARMTD demo from MATLAB using `run_fetch_hardware.m`