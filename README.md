Clone all the file to ~/[catkin_workspace]/src

The file cantains 3 example

# First example
 talker.py and listener.py files belang to the first example if you want to run this example 
 
 1. first you need to clone in you machine
 
 2. make the filse executable using linux cmd 
   chmod +x talker.py
   chmod +x listener.py
   
 3. $ cd ~/catkin_ws
    $ catkin_make
    
 4. run the roscore using following cmd (open new terminal)
 $ roscore
 
 5. Running the Publisher (open new terminal)
  catkin specific If you are using catkin, make sure you have sourced your workspace's setup.sh file after calling catkin_make but  before trying to use your applications: 
 In your catkin workspace
$ cd ~/catkin_ws
$ source ./devel/setup.bash
 
 $ rosrun beginner_tutorials talker      (C++)
$ rosrun beginner_tutorials talker.py   (Python) 

Running the Subscriber(open new terminal)

$ rosrun beginner_tutorials listener     (C++)
$ rosrun beginner_tutorials listener.py  (Python) 
 
 
 # Second Example (Move Turtlebot in Gazebo world)
 after clone the [turtlebot] folder  to ~/your catkin_workspace/src
 1. launch turtlebot in gazebo world by typing following cmd n terminal 
   $ roslaunch turtlebot_gazebo turtlebot_world.launch
 2. run goforword.py (new terminal)
    $ cd ~/catkin_ws/src/turtlebot/scripts
    $ python goforward.py
    pythone
 see turtlebot is moving
