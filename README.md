# Introduction to Robotics (DD2410)

## Assignment 1: Introduction to ROS

In this course we will be using the Robot Operating System (ROS). ROS is a middleware platform that provides libraries and tools that help software developers create robot applications. It simplifies the integration of different components on the same system and over the network.

### Why do we use ROS?

![ROS community map](images/ros_community.png "ROS community map")

* Very large user community
* Standard in many robotics labs around the world, even in some companies
* Many commercially available robots use ROS nowadays
* Open source + large community = lots of packages, libraries, and tools available
  * Robot planning and control
  * Navigation
  * Visualization tools
  * Hardware drivers and interfaces
  * Etc ...
* Modularization and abstraction
* Standardization/structure
* Easier to collaborate with others
* Make roboticists' life easier

### Basic ROS concepts

ROS has a great wiki that you can find [here](http://wiki.ros.org/). There you can find basically everything about ROS, and it should be the first place you look when you have trouble with ROS. There is also a Q&A, like Stack Overflow but for ROS, [here](https://answers.ros.org/) where you can ask questions and find answers to other people questions.

Since ROS already has a great wiki with tutorials and such, we will only briefly mention the most basic concepts here. You can then find more information in the ROS wiki as you please. We think this is a good way of teaching you ROS, since the most important skill for you in order to master ROS is to be able to find the information you need fast in the ROS wiki.

* [Nodes](http://wiki.ros.org/Nodes): ROS nodes are executables, processes that perform computation. A node should perform a single well-defined task, such as motor control, localization, sensor processing, etc. In a typical system you have many nodes. Each running node is able to exchange information through topics and services. A ROS node is written with the use of a ROS [client library](http://wiki.ros.org/Client%20Libraries), such as [roscpp](http://wiki.ros.org/roscpp) or [rospy](http://wiki.ros.org/rospy). In this course you will write all code in Python so you will be using rospy. Remember to [make your Python script executable](https://superuser.com/a/828740).
* [Master](http://wiki.ros.org/Master): The ROS Master is what allows nodes to be able to find each other and exchange messages, or invoke services. For a functional ROS system you have to have a ROS master running. To explicitly start the ROS Master you use the command ```roscore```. A ROS Master is implicitly started when running the command ```roslaunch```, if there is no ROS Master already running. We recommend that you always explicitly start the ROS Master, such that you do not have to restart the whole system when restarting the _launch file_.
* [Paremeter Server](http://wiki.ros.org/Parameter%20Server): The Parameter Server allows you to store data as key-value pairs, which nodes can access. This is especially useful when writing nodes in C++, since it allows you to change a parameters value without recompiling the node. The Parameter Server is part of the ROS Master.
* [Messages](http://wiki.ros.org/Messages): Nodes communicate with each other by passing messages. A message is simply a data structure that can be passed between nodes.
* [Topics](http://wiki.ros.org/Topics): Nodes can send out messages by _publishing_ the messages to a topic. Another node can then _subscribe_ to the same topic in order to recieve the messages. For example, a camera node would publish camera images to some topic, a vision system can then subscribe to that topic in order to get the images. Topics implement many-to-many relationship, meaning there can be multiple publishers and subscribers on the same topic. For each topic there can only be one type of message that is being published. A single node can subscribe and publish to multiple different topics.
* [Services](http://wiki.ros.org/Services): The publish and subscribe model of topics is very flexible, sometimes, however, you want to be able to request something and get a response. Services implement a client-server relationship. One node here acts as a server and offers the service. Other nodes (or the same node) acts as clients and can use the service. A service is defined by a pair of messages, the _request_ message and the _response_ message. A service call is almost like a function call. You might have a path planning node that is offering a path planning service. Another node can then call that path planning service with a starting point and an end point as the request message and will in return get a response message containing the path from the starting point to the end point.
* [Bags](http://wiki.ros.org/Bags): Bags are a format for recording and playing back ROS message data. They allow you to test your algorithms on data without running the system. This can be espeically useful when you are working in places where there is difficult or time consuming to collect new data everytime you want to test a change to your algorithms.
* [Packages](http://wiki.ros.org/Packages): pieces of software in ROS are bundled together and distributed through packages. They contain source code for nodes, message/service/parameters files, and build and install instructions.

You can (and probably should) read more about the [basic ROS concepts](http://wiki.ros.org/ROS/Concepts) in the ROS wiki.

### Higher level ROS concepts

Here we will describe some of the higher level ROS concepts. You can (and probably should) read more about the [higher level ROS concepts](http://wiki.ros.org/ROS/Higher-Level%20Concepts) in the ROS wiki.

#### [TF](https://wiki.ros.org/tf2)

Let's first take a step back and look at how to deal with coordinate systems in ROS. A coordinate system is defined by a frame of reference. You might for example say that your computer is 50 cm from the left edge of your table, and 40 cm from the bottom edge. This coordinate system uses the table's lower left corner as its frame of reference (its origin), but clearly, you could choose any arbitrary reference frame. Perhaps it would make more sense to locate your laptop with respect to the north-east corner of the room you're in. If you then know where the table is in the room, it should be trivial to calculate the position of your laptop in the room -- this is exactly what TF does for you. A more concretely robotics-related application is available in section 1 of [http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF). More exact terminology can be found on the ROS wiki's [TF terminology page](http://wiki.ros.org/tf2/Terminology).

In mobile robotics there are three important frames that (almost) always are present when working with ROS: the map frame, considered the "absolute" coordinate system in that it doesn't change over time; the odom frame, for odometry frame, whose origin typically is where the robot was powered on; and the base_link frame, which is the robot frame -- in other words, the robot is always at the origin of the base_link frame. REP 105 (Links to an external site.) defines these, and also gives some naming conventions and semantics, as well as some additional common frames.

The picture below illustrates the relationship between the three frames mentioned above and some additional ones. A position x=(x, y, z) or a pose (x, y, z and three rotations about the body axes) can be expressed in any of the frames. However, one of them is usually more natural than the other. For example, your laptop is easier to locate with respect to the table than the room in the example above. In the image below, the location of the landmarks L1 and L2 are easier to express in the map frame, whereas the position of the camera_link is defined with respect to base_link (i.e. relative to the robot.) We can see from the graph that in order to know where the camera is in the map frame we also need to know where the robot is in the odom frame and the relation between the odom frame and the map frame. This requires a localization system, which estimates the pose of the robot in the map frame and therefore calculates the transform between map and odometry. We will see later how TF allows us to seamlessly move from frame to frame and thus make it possible to, for example, express the location of the landmarks in the base_link frame.

![TF tree handdrawn](images/TF_tree_big_handdrawn.jpg "TF tree handdrawn")

We recommend you to use the newer [TF2](https://wiki.ros.org/tf2) instead of the old [TF](https://wiki.ros.org/tf). However, things such as converting a quaternion to/from euler has not yet been implemented for the Python API of TF2, therefore you have to use TF in these situations. You can see an example of that [here](https://wiki.ros.org/tf2/Tutorials/Quaternions#Think_in_RPY_then_convert_to_quaternion).

#### [Actionlib](http://wiki.ros.org/actionlib)

Services are great, however sometimes you want to cancel a request to a service. For example if the service is taking too much time or if you noticed something in the environment that changes what you want to do.

It would also be great if you could get periodical feedback about the request. For example if you have a path planner you might be able to start moving along the path even before the whole path has been calculated.

This is where _actionlib_ comes in. It is like _services_ with the added ability for the client to cancel the request and for the server to send periodic feedback to the client.

Use __simple__ action server and client.

<!-- #### [RQT](https://wiki.ros.org/rqt) -->

<!-- #### [RViz](https://wiki.ros.org/rviz) -->

<!-- #### [Robot Model](http://wiki.ros.org/urdf) -->

### ROS cheat sheet

You can find a useful ROS cheat sheet [here](https://github.com/ros/cheatsheet/releases/). As you can see it is for ROS Indigo, however everything seems to be the same for Melodic.

### When you need help with ROS

There are a lot of nice tutorials and information on the ROS webpage. Always look there first for information on ROS/ROS Packages.

* [ROS wiki](https://wiki.ros.org/)
* [ROS Q/A](https://answers.ros.org/)
* [ROS tutorials](https://wiki.ros.org/ROS/Tutorials/)

## The practical part of the assignment

Now we will start with the practical part of the assignment.

### What we use in this course

* Ubuntu __18.04__
* ROS __Melodic__
* Python
  * Version 2.7
  * Default with Ubuntu 18.04

If you are interested you can read more [here](http://www.ros.org/reps/rep-0003.html).

Everything is installed for you in computer labs:

* E building: Röd, Orange, Gul, Grön, Brun, Grå, Karmosin, Vit, Magenta
* D bulding: Spel, sport

### Install, source, and create ROS workspace

[Install ROS (ros-melodic-desktop-full)](https://wiki.ros.org/melodic/Installation/Ubuntu), __you should skip this step if you are using the computers in the computer labs mentioned above__. You also need:

```bash
sudo apt install ros-melodic-ros-tutorials ros-melodic-turtlebot3 ros-melodic-turtlebot3-simulations ros-melodic-navigation libspatialindex-dev
pip install rtree, sklearn
```

[Source ROS](https://wiki.ros.org/melodic/Installation/Ubuntu#melodic.2BAC8-Installation.2BAC8-DebEnvironment.Environment_setup):

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

[Create a ROS workspace](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment):

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Recommended editor

I recommend that you use [VS Code](https://code.visualstudio.com/) with the extensions:

* [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
* [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
* Other that you find useful...

### Tutorials

#### Basic ROS tutorials

In order for you to get a practical understanding of ROS you should now do the beginner level core ROS tutorials that you can find [here](https://wiki.ros.org/ROS/Tutorials#Core_ROS_Tutorials). It may seem like a lot to do, but it really is not. You mostly just copy-paste and see what happens, so please spend some time and think of what you are actually doing and what is happening.

We did 1 above so you can skip that one if you want. Since we are using Python you should therefore do:

* [2. Navigating the ROS Filesystem](https://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)
* [3. Creating a ROS Package](https://wiki.ros.org/ROS/Tutorials/CreatingPackage)
* [4. Building a ROS Package](https://wiki.ros.org/ROS/Tutorials/BuildingPackages)
* [5. Understanding ROS Nodes](https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
* [6. Understanding ROS Topics](https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
* [7. Understanding ROS Services and Parameters](https://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
* [8. Using rqt_console and roslaunch](https://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
* [9. Using rosed to edit files in ROS](https://wiki.ros.org/ROS/Tutorials/UsingRosEd)
* [10. Creating a ROS msg and srv](https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
* [12. Writing a Simple Publisher and Subscriber (Python)](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
* [13. Examining the Simple Publisher and Subscriber](https://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)
* [15. Writing a Simple Service and Client (Python)](https://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)
* [16. Examining the Simple Service and Client](https://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient)
* [17. Recording and playing back data](https://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data)
* [18. Getting started with roswtf](https://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf)
* [19. Navigating the ROS wiki](https://wiki.ros.org/ROS/Tutorials/NavigatingTheWiki)

If you want you may also do the intermediate level tutorials and/or the C++ tutorials.

#### TF2 tutorials

The TF library in ROS is very useful when you are building real systems. Therefore we feel it is important that you get to try it out. You should therefore do the [TF2 tutorials](https://wiki.ros.org/tf2/Tutorials) for Python.

You should do at least:

* [1. Introduction to tf2](https://wiki.ros.org/tf2/Tutorials/Introduction%20to%20tf2)
* [1. Writing a tf2 static broadcaster (Python)](https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28Python%29)
* [2. Writing a tf2 broadcaster (Python)](https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29)
* [3. Writing a tf2 listener (Python)](https://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29)
* [4. Adding a frame (Python)](https://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28Python%29)
* [5. Learning about tf2 and time (Python)](https://wiki.ros.org/tf2/Tutorials/tf2%20and%20time%20%28Python%29)
* [6. Time travel with tf2 (Python)](https://wiki.ros.org/tf2/Tutorials/Time%20travel%20with%20tf2%20%28Python%29)
* [1. Quaternion Basics](https://wiki.ros.org/tf2/Tutorials/Quaternions)

You can of course do the other tutorials as well.

#### Actionlib tutorials

Actionlib is useful when you have something that you want to function as a service but with the abilities to send intermediate results and cancel the request.

An example for when actionlib is useful. Imagine that you have a path following node that provides a path following _service_. Once another node has made a request on the path following _service_ the path planning node will execute the whole path. But what happens if you see something along the way and want to do something else based on that? Well, if you use a _service_ you cannot do much other than wait until the path follower has reached the end of the path. If the path following node instead had provided a path following _action_ then the other node would have been able to cancel the request once you saw something interesting, and the robot would therefore stop following the path.

You should do at least these actionlib tutorials:

* [4. Writing a Simple Action Server using the Execute Callback (Python)](https://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29)
* [5. Writing a Simple Action Client (Python)](https://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29)

### Mini-project

#### Installation

```bash
cd ~/catkin_ws/
wstool init src
cd src
wstool set irob_assignment_1 --git https://github.com/danielduberg/irob_assignment_1.git -v master
wstool set hector_slam --git https://github.com/tu-darmstadt-ros-pkg/hector_slam.git -v melodic-devel
wstool update
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
source ~/.bashrc
```

#### Description

As you all know: Exploration is the ___coolest___ area of robotics. For your first ROS project you should therefore help a robot that is almost ready to perform exploration.

If you launch the launch file `start.launch` inside `irob_assignment_1/launch`:

```bash
roslaunch irob_assignment_1 start.launch
```

You will see this:

![RViz view](images/rviz.png "RViz view")

Take a look at the TF tree by running the command `rqt`, to start RQT, then in the top bar select `Pluings->Visualization->TF Tree`:

![TF tree](images/frames.png "TF tree")

Take a look at the node graph in RQT `Plugins->Introspection->Node Graph` and uncheck `Leaf topics`:

![Node graph](images/rosgraph.png "Node graph")

#### Simple approach

Pseudocode of the assignment:

```python
# Init stuff
while True:
    path, gain = get_path_from_action_server()
    if path is empty:
        exit() # Done
    while path is not empty:
      path, setpoint = get_updated_path_and_setpoint_from_service(path)
      setpoint_transformed = transform_setpoint_to_robot_frame(setpoint)
      publish(setpoint_transformed)
      sleep()
```

#### [OPTIONAL] Callback based approach

Minimal example of __Callback Based SimpleActionClient__:

```python
#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg

goal_client = None

def goal_active():
    pass


def goal_feedback(feedback):
    pass


def goal_result(state, result):
    if actionlib.TerminalState.SUCCEEDED == state:
        pass
    elif actionlib.TerminalState.RECALLED == state:
        pass
    elif actionlib.TerminalState.REJECTED == state:
        pass
    elif actionlib.TerminalState.PREEMPTED = state:
        pass
    elif actionlib.TerminalState.ABORTED = state:
        pass
    elif actionlib.TerminalState.LOST = state:
        pass


def get_path():
    global goal_client
    goal_client.wait_for_server()
    goal = irob_assignment_1.msg.GetNextGoalGoal()
    goal_client.send_goal(goal, active_cb=goal_active,
                          feedback_cb=goal_feedback, done_cb=goal_result)


if __name__ == "__main__":
    rospy.init_node("controller")

    # Action client
    goal_client = actionlib.SimpleActionClient(
        "get_next_goal", irob_assignment_1.msg.GetNextGoalAction)

    get_path()

    rospy.spin()
```

How can you use the __Callback Based SimpleActionClient__ in order to increase the speed of the exploration? You can implement the assignment with __Callback Based SimpleActionClient__ in __less than 100 lines of Python code__.

What you should see if you have done everything correct:

[![Finished mini-project](http://img.youtube.com/vi/kexHizTs5M4/0.jpg)](http://www.youtube.com/watch?v=kexHizTs5M4)


#### The presentation

The important part of this assignment is not that you write superb code. What we want you to focus on is to understand ROS.

You should be able to explain the different ROS concepts, what nodes, topics, services, messages, actionlib, ect. You should be able to explain all lines in your code: why do you need a `rospy.init_node(...)`? What does `pub = rospy.Publisher(...)` and `pub.publish(...)` do? What is the `queue_size` parameter used for when you are calling `rospy.Publisher(...)`?

#### (Will be) Frequently Asked Questions

##### Why is exploration the coolest area of robotics?

Say one area in robotics that is cooler? There you go.

##### Which TF functions should I use to complete the mini-project?

I did it with `tf_buffer.lookup_transform(...)` and `tf2_geometry_msgs.do_transform_point(...)`.

##### My robot moves weird in the mini-project, why?

Did you set a maximum linear and angular velocity? 0.5 and 1.0, respectively, should work.

##### How do I get the angular velocity?

Math. Use: `math.atan2(y, x)`.

##### The TF says something about extrapolating into the past, why?

Probably because you did not initilize the TF buffer such that it has the transforms, like this:

```python
# Imports

# Global variables

tf_buffer = None
listener = None

# Your other functions

if __name__ == "__main__":
    rospy.init_node("controller")

    # Other stuff you want to init

    # Create TF buffer
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Function calls and such

    rospy.spin()

```

You should do this in the beginning (right after `rospy.init_node(...)`) such that it has time to buffer the transforms.

##### The TF says something about extrapolating into the future, why?

Probably because you are trying to get a transform that is not yet available. TF2 cannot extrapolate, only interpolate.

You can use these inside `tf_buffer.lookup_transform(...)`:
`rospy.Time(0)` to get the most recent transform and `rospy.Duration(1)` to wait for 1 second if that is needed.

##### Why is the exploration so slow?

You probably did not use the Callback Based SimpleActionClient. Implement it using Callback Based SimpleActionClient instead. In the feedback callback you can see if the current `gain` is over, for example, 3 (or some other value that you find is good) and then cancel the goal if that is the case.

##### I am trying to cancel the goal but it is not working :(

Try to cancel all the goals using `goal_client.cancel_all_goals()`.

##### I implemented the Callback Based SimpleActionClient, but it is not working :(

Did you actually cancel the request after you got a feedback with high enough gain? When you cancel the request you will get a result `actionlib.TerminalState.PREEMPTED`, so be sure that in the result callback that you only do something if the state is `actionlib.TerminalState.SUCCEEDED`.

##### It is too much to do and it is too difficult :(

It really is not that much to do. The tutorials are just reading and copy-paste. I solved the mini-project with Callback Based SimpleActionClient using 3 if-statements and 1 while-loop. The rest was basically just copy-paste from the tutorials. You find it hard because you are new to ROS, in 1 year you will look back and laugh at yourself.
