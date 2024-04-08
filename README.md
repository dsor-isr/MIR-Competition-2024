# MIR Competition 2024 

In the 2024 edition of the MIR challenge, you will have the oportunity to design a controller for a high velocity research Autonomous Surface Vehicle (ASV)...

# Instalation Guidelines:

#### At the end of these instructions, your project folder should have the following structure:
![folder_struct_glassy_challenge](https://github.com/joaolehodey/MIR-Competition-2024/assets/69345264/d5a62ac1-5abb-4a4c-bb2e-b6bccc614ae8)

The "MyProject" folder, constains 4 different directories with the following objectives:
* PX4-Autopilot: This folder contains a fork of the original PX4-Autopilot, with the addition of a custom made simulation of our ASV. Our ASV is equiped with a Pixhawk running PX4-Autopilot. The PX4-Autopilot is used to to its out-of-the-box sensor integration.
* QGroundControl: This Folder will contain the file QGroundControl.AppImage. The QGroundControl application is oppened either by doubleclicking the file or executing the file in a terminal. The previous is used to monitor the state of the vehicle from the 'Ground station', its is used to change the current vehicle mode, check its position and orientation on the map, ...
* Micro-XRCE-DDS-Agent: This folder contains the Micro XRCE DDS Agent, which is used to bridge the u-orb topics (internal PX4 information topics) to Robot Operating System 2 (ROS2) Topics.
* glassy_challenge_ws: This folder contains software developed to communicate, monitor and activate missions. Is is structured following the usual ROS2 structure. In this case, the mission that will be activated is the challenge mission, where your own controller will run.

For the challenge, the only file that needs to be changed is "glassy_challenge.py".

### Step-by-step instructions:

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
    
* Create your Project Folder, in this case we will name it "MyProject", and enter the newly created folder:
  ```console
  cd
  mkdir MyProject && cd MyProject
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
  git checkout glassy_simulation
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
  cd ~MyFolder/PX4-Autopilot
  make px4_sitl gazebo-classic_glassy
  ```
  An ocean world should appear, with a green RC boat present in the middle of it.

* Create the QGroundControl folder:
  ```console
  cd ~/MyProject
  mkdir QGroundControl
  ```
* Install QgroundControl following the Ubuntu installation instructions: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html, please place the QGroundControl.AppImage file downloaded in the MyProject/QGroundControl folder. Ensure everything is working, by ensuring you can open the app (as explained in the download and install tutorial).
  
* Re-enter the project folder:
    ```console
  cd ~/MyProject
  ```
Follow the instructions to install the uXRCE-DDS (PX4-ROS 2/DDS Bridge): https://docs.px4.io/main/en/middleware/uxrce_dds.html#install-standalone-from-source
Ensure everything is working by running starting the agent and checking for any warning or error:
```console
   MicroXRCEAgent udp4 -p 8888
  ```

* Again, return to the project folder:
  ```console
  cd ~/MyProject
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
  git clone https://github.com/joaolehodey/summer_challenge_IST_DSOR.git .
  ```
* Clone the px4_msgs ros2 package:
  ```console
  git clone https://github.com/PX4/px4_msgs.git
  ```

* Enter the px4_msgs directory and checkout the correct branch:
  ```console
  cd px4_msgs
  git checkout release/1.14
  ```
  

  
* Alter the .bashrc to source both ros2 and the ros2 workspace. This can be done by running the following commands.
  ```console
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc 
  echo  'source ~/MyProject/glassy_px4_ws/install/setup.bash' >> ~/.bashrc 
  ```
* **Restart your computer**
    ```console
  reboot
  ```

 #### The instalation is now complete, to ensure everything is working:

* Open a new terminal and start the Micro XRCE-DDS Agent agent:
  ```console
  MicroXRCEAgent udp4 -p 8888
  ```
* Open a new terminal and navigate to the PX4-Autopilot directory, then start the simulation (the first time will take longer):
    ```console
  make px4_sitl gazebo-classic_glassy
  ```
* Open a new terminal and navigate to the glassy_challenge_ws directory, then build the ROS2 code (the first time will take longer):
    ```console
  colcon build
  ```
* When the building process is finished, run the glassy_manager node:
    ```console
  ros2 run glassy_px4_manager glassy_px4_manager
  ```
 * Open a new terminal and run the glassy_challenge node:
  ```console
    ros2 run glassy_challenge glassy_challenge
  ```

### MUST ADD SCREENSHOTS OF EXPECTED RESULTS ,...


### Additionally, some of the following tools may be usefull:
* Plotjuggler: https://github.com/facontidavide/PlotJuggler
* Terminator: https://gnome-terminator.readthedocs.io/en/latest/ (***Highly Recomended***)


## Start developing your solution:
As mentioned previously, to complete the challenge only the file 'glassy_challenge.py' inside 'ProjectFolder/glassy_challenge_ws/src/glassy_challenge/glassy/challenge' needs to be changed.
More specifically, you should only make changes in the function 'myChallengeController' and, if you feel the need to keep track of more variables, those should be initialized in the class contructor.

## In case you have any questions:
-- insert here contacts, ...
-- relatively new system, some bugs may occur, do not hesitate to contact us

## 
