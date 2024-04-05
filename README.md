# 2024 Summer challenge at IST 

...
...
...

# Instalation Guidelines:

#### At the end of these instructions, your project folder should have the following structure:
![folder_struct_glassy_challenge](https://github.com/joaolehodey/summer_challenge_IST_DSOR/assets/69345264/1022c432-65f5-478e-a54b-57e3fdc693a4)

For the challenge, the only file that needs to be changed is "glassy_challenge.py".

#### Step-by-step instructions:
Please ensure you have Ubuntu 22.04 installed on your machine.
* Install ROS2 Humble by following the instructions on https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
* Install Gazebo 11 by following the instructions on: https://classic.gazebosim.org/tutorials?tut=install_ubuntu
* Create your Project Folder, in this case we will name it "MyProject", and enter the newly created folder:
  ```console
  mkdir MyProject && cd MyProject
  ```
* Clone the PX4 repository fork and switch to correct branch, you may do so by following the steps presented below:
  ```console
  git clone --recurse-submodules -j8 https://github.com/joaolehodey/PX4-Autopilot
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

* Install QgroundControl following the Ubuntu installation instructions: https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
* Inside ProjectFolder, follow the instructions to install the uXRCE-DDS (PX4-ROS 2/DDS Bridge): https://docs.px4.io/main/en/middleware/uxrce_dds.html#install-standalone-from-source

* Inside a terminal, enter the ProjectFolder directory,
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
  git clone https://github.com/joaolehodey/summer_challenge_IST_DSOR.git
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

  
* Alter the .bashrc to source both ros2 and the ros2 workspace. This can be done by apending the following lines to the .bashrc file, (replace the '\*path_to_ProjectFolder_parent_directory\*' by the correct path.
  ```console
  source /opt/ros/humble/setup.bash
  source ~/*path_to_ProjectFolder_parent_directory*/ProjectFolder/glassy_px4_ws/install/setup.bash
  ```
* **Restart your computer**

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
