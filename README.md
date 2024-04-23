# MIR Competition 2024 

In the 2024 edition of the MIR challenge, you will have the oportunity to design a controller for a high velocity Autonomous Surface Vehicle (ASV).

# Instalation Guidelines:

#### At the end of these instructions, your project folder should have the following structure:
![folder_struct_glassy_challenge](https://github.com/joaolehodey/MIR-Competition-2024/assets/69345264/8d671db8-3be0-41e7-8901-ec253e136641)

The "MIR_Project_2024" folder, constains 4 different directories with the following objectives:
* PX4-Autopilot: This folder contains a fork of the original PX4-Autopilot, with the addition of a custom made simulation of our ASV. Our ASV is equiped with a Pixhawk running PX4-Autopilot. The PX4-Autopilot is used to to its out-of-the-box sensor integration.
* QGroundControl: This Folder will contain the file QGroundControl.AppImage. The QGroundControl application is oppened either by doubleclicking the file or executing the file in a terminal. The previous is used to monitor the state of the vehicle from the 'Ground station', its is used to change the current vehicle mode, check its position and orientation on the map, ...
* Micro-XRCE-DDS-Agent: This folder contains the Micro XRCE DDS Agent, which is used to bridge the u-orb topics (internal PX4 information topics) to Robot Operating System 2 (ROS2) Topics.
* glassy_challenge_ws: This folder contains software developed to communicate, monitor and activate missions. Is is structured following the usual ROS2 structure. In this case, the mission that will be activated is the challenge mission, where your own controller will run.

For the challenge, the only file that needs to be changed is "glassy_challenge.py".
The file CHALLENGE.pdf contains information about the vehicle you will use, its model and the challenge objectives.

( ENSURE YOU HAVE A GOOD WIFI CONNECTION DURING THE INSTALLATION, THE FULL SETUP SHOULD TAKE AT LEAST 30 MIN )

### Option 1: Script Installation (RECOMMENDED)

#### System requirements:
Please ensure you have Ubuntu 22.04 installed on your machine. We **highly** recommend a native instalation (https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview), altough you may also install Ubuntu 22.04 on a Virtual Machine (VMware, Virtualbox, etc.).
Please note that PCs/Macs with ARM architecture are not supported due to missing dependencies needed for both PX4-Autopilot Software-In-The-Loop (PX4 SITL) and the Gazebo Simulator.
In a terminal, run:
```console
sudo apt-get update
sudo apt-get upgrade
```
```console
sudo snap install curl
```
In a terminal, run:
```console
curl -L https://raw.githubusercontent.com/joaolehodey/MIR-Competition-2024_instalation_script/main/challenge.bash | bash
```
### Option 2: Step-by-step instructions:
#### System requirements:
Please ensure you have Ubuntu 22.04 installed on your machine. We **highly** recommend a native instalation (https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview), altough you may also install Ubuntu 22.04 on a Virtual Machine (VMware, Virtualbox, etc.).
Please note that PCs/Macs with ARM architecture are not supported due to missing dependencies needed for both PX4-Autopilot Software-In-The-Loop (PX4 SITL) and the Gazebo Simulator.

#### Installation Procedure:
* Install ROS2 Humble by following the instructions on https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html (just copy and paste the terminal commands in the yellow boxes), make sure you install the desktop version (install the recommended version instead of the bare-bones version). Install the Development tools as well.
Before moving forward, please ensure that ROS2 is properly installed by experimenting with the examples present in the ROS2 installation tutorial.

* Install the colcon tools:
  ```console
  sudo apt install python3-colcon-common-extensions
  ```
    
* Create your Project Folder, in this case we will name it "MIR_Project_2024", and enter the newly created folder:
  ```console
  cd
  mkdir MIR_Project_2024 && cd MIR_Project_2024
  ```
* Clone the PX4 repository fork and switch to correct branch, you may do so by following the steps presented below:
  ```console
  git clone --recurse-submodules https://github.com/joaolehodey/PX4-Autopilot
  ```
  ```console
  cd PX4-Autopilot
  ```
  ```console
  make clean
  ```
  ```console
  make distclean
  ```
  ```console
  git checkout glassy_MIR
  ```
  ```console
  make submodulesclean
  ```
Finally, run the setup script in order to install of the PX4-Autopilot dependencies (From inside the PX4-Autopilot directory):
  ```console
  bash ./Tools/setup/ubuntu.sh
  ```
* Install Gazebo 11 by following the instructions on: https://classic.gazebosim.org/tutorials?tut=install_ubuntu, we recommend the one-line installer. After installing, run the command:
    ```console
  gazebo
  ```
And ensure that the gazebo simulator opens, in an empty world.

* Ensure the PX4 and the custom simulation by entering the PX4-Autopilot folder and starting the simulation:
  ```console
  cd 
  MIR_Project_2024/PX4-Autopilot
  make px4_sitl gazebo-classic_glassy
  ```
  An ocean world should appear, with a green RC boat present in the middle of it.

* Create the QGroundControl folder:
  ```console
  cd ~/MIR_Project_2024
  mkdir QGroundControl
  ```
* Install QgroundControl following the Ubuntu installation instructions: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html, please place the QGroundControl.AppImage file downloaded in the MIR_Project_2024/QGroundControl folder. Ensure everything is working, by ensuring you can open the app (as explained in the download and install tutorial).
  
* Re-enter the project folder:
    ```console
  cd ~/MIR_Project_2024
  ```
Follow the instructions to install the uXRCE-DDS (PX4-ROS 2/DDS Bridge): https://docs.px4.io/main/en/middleware/uxrce_dds.html#install-standalone-from-source
Ensure everything is working by running starting the agent and checking for any warning or error:
```console
   MicroXRCEAgent udp4 -p 8888
  ```

* Again, return to the project folder:
  ```console
  cd ~/MIR_Project_2024
  ```
* Create the ROS2 workspace folder and enter the newly created folder:
  ```console
  mkdir glassy_challenge_ws && cd glassy_challenge_ws
  ```
* Create the src folder and enter the newly created folder:
  ```console
  mkdir src && cd src
  ```
* Clone the challenge code:
  ```console
  git clone git@github.com:dsor-isr/MIR-Competition-2024.git .
  ```
* Clone the px4_msgs ros2 package:
  ```console
  git clone https://github.com/PX4/px4_msgs.git
  ```

* Install the Eigen c++ library:
  ```console
  sudo apt install libeigen3-dev
  ```    
  
* Alter the .bashrc to source both ros2 and the ros2 workspace. This can be done by running the following commands.
  ```console
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc 
  echo  'source ~/MIR_Project_2024/glassy_challenge_ws/install/setup.bash' >> ~/.bashrc 
  ```
* Update the scipy python package:
    ```console
  pip install scipy --upgrade
  ```

* **Restart your computer**
    ```console
  reboot
  ```

 # The instalation is now complete, lets put the whole system working for the first time:
* Open a new terminal and compile the ROS2 workspace:
    ```console
  cd ~/MIR_Project_2024/glassy_challenge_ws
  colcon build 
  ```
  You may close this terminal, or use it for the next task.
  
* Open a new terminal (or use the previous one) and start the Micro XRCE-DDS Agent agent:
  ```console
  MicroXRCEAgent udp4 -p 8888
  ```
* Open a new terminal and navigate to the PX4-Autopilot directory, then start the simulation (the first time will take longer):
    ```console
  cd ~/MIR_Project_2024/PX4-Autopilot
  make px4_sitl gazebo-classic_glassy
  ```
    You should get the following:
![image](https://github.com/joaolehodey/MIR-Competition-2024/assets/69345264/c3120b5d-fc32-40cb-ada1-d2f759d23376)

* Open a new terminal, run the glassy_px4_manager node:
    ```console
  ros2 run glassy_px4_manager glassy_px4_manager
  ```
 * Open a new terminal and run the glassy_challenge node:
  ```console
    ros2 run glassy_challenge glassy_challenge
  ```
* Open the QGroundControl App, this can be done by simply double clicking the QGroundControl.AppImage

After oppening Qgroundcontrol, a connection should occur and you should see that the vehicle is connected to the app:
![image](https://github.com/joaolehodey/MIR-Competition-2024/assets/69345264/6a48337f-8ef3-46f7-ac4c-6be6b6c59985)







## Workflow and solution development:

### Solution development:
As mentioned previously, to complete the challenge only the file 'glassy_challenge.py' inside 'ProjectFolder/glassy_challenge_ws/src/glassy_challenge/glassy/challenge' needs to be changed.
More specifically, you should only make changes in the function 'myChallengeController' and, if you feel the need to keep track of more variables, those should be initialized in the class contructor.

### Workflow:

Open 4 distict terminals.

* In the first terminal run the MicroXRCEAgent:
```console
  MicroXRCEAgent udp4 -p 8888
```
You can keep the agent running in the background, and not worry about it.

* In a second terminal, start the simulation:
  ```console
  cd ~/MIR_Project_2024/PX4-Autopilot
  make px4_sitl gazebo-classic_glassy
  ```
You can also leave this running, altough sometimes, it is necessary to restart the simulation (if sensor/multicast errors occur, ... its usually good to restart the simulation)

* In a third terminal start the glassy_px4_manager node:
    ```console
  ros2 run glassy_px4_manager glassy_px4_manager
  ```
Simillarly to the above, this can keep running in the background.

* Open QGroundControl, do this by double-clicking the QGroundControl.AppImage file (keep it open during your development).

* Now, write your code in the 'glassy_challenge.py' file. When you are done and ready to test it, build the code and run the glassy_challenge node:
     ```console
  cd ~/MIR_Project_2024/glassy_challenge_ws
  colcon build --packages-select glassy_challenge #this builds your code
  ros2 run glassy_challenge glassy_challenge  # this runs your code
  ```


Your code is running, however nothing is happening, this is because you need to start the mission, to start the mission, you must enter Offboard mode **AND** Arm the vehicle. Arming corresponds in a way to turn on the motors (allowing them to spin). Offboard mode is an internal PX4 mode, where PX4 allows an outside source to publish commands (in our case it is an onboard computer). The first time you do this (before you write any code), you should see the vehicle moving forward in zigzags (constant thrust and sinusoidal rudder input).

To do both of the above you need to use QgroundControl:
![Your How to Chnage Mode tutorial](https://github.com/joaolehodey/MIR-Competition-2024/assets/69345264/f5825b22-a799-4b22-8411-90815c7c9de1)
![Your How to Arm text](https://github.com/joaolehodey/MIR-Competition-2024/assets/69345264/29a88eb8-36bf-465c-a6cd-9142a097353c)

***IMPORTANT:*** Please note that this simulation is very similar to the real system, including the failsafes. Failsafes are a set of conditions that allow the arming of the vehicle. These ensure the vehicle is not armed if there are sensor failures, no access to manual control, etc. Given the above, it is important that you have a source of manual inputs to act as your manual control, otherwise you will not be able to arm the vehicle. If you have a joystick (PS4 controller, ...) , plug it into your machine, it should be detected by QGroundControl, and will allow the arming of the vehicle. You may also drive the vehicle around by entering Manual Mode (check the change mode tutorial above). If you do not have a joystick around, you may activate the QGroundControl virtual joystick ( it does not really allow proper manual control, but is usefull to allow the arming of the vehicle for developement purposes). To activate the Qgroundcontrol virtual joystick:

![Turn_virtual_joystick_on](https://github.com/joaolehodey/MIR-Competition-2024/assets/69345264/712050cf-608b-4dc6-81f6-9840072d41cd)

**Finally, the information about the controller you need to design and the model of the vehicle can be found in the CHALLENGE.pdf file.**

### Additionally, some of the following tools may be usefull to help you:
* Plotjuggler: https://github.com/facontidavide/PlotJuggler (***Highly Recomended For Plotting and Debugging***)
  - Install using: sudo apt install ros-humble-plotjuggler-ros
  - Run using: ros2 run plotjuggler plotjuggler
* Terminator: https://gnome-terminator.readthedocs.io/en/latest/ (***Highly Recomended***)

### If you want to know more about the software ecosystem used:
* PX4 website: https://docs.px4.io/main/en/
* ROS 2 website: https://docs.ros.org/en/humble/index.html


## In case you have any questions:
If you think there is a bug or something is not clear enough, post an issue on github, this way everybody can see it and benefit from it. 
