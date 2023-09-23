# Step 1: Setup

Open this workspace in a fresh terminal and run the following command to set up any dependencies

    ./setup.sh

# Step 2: Edit code

In publisher_node.cpp, create an instance of publisher node and pass it to rclcpp::spin, as you did in the previous exercise. This is now running with the vanilla executor and this node won't have a real-time priority.

// TODO: Do this one by default and let the audience modify sensor_node?
In actuation_node.cpp, instead of editting the subscription, set the thread priority directly in the main function using a call to sched_setscheduler. Give it a priority of 50 and use the SCHED_FIFO policy

In sensor_node.cpp, there are two callbacks, which will run in different threads. The data logger will run in the default thread with no real-time priority, but the object detector will run in a separate thread with a real-time priority on the SCHED_FIFO scheduler.

??? Callback groups

You'll need two executors, one for the data logger, which has been created for you, and a second for the object detector. Make this.

Create a separate thread using a lambda function and pass in the object detecting executor. In that thread make a call to sched_setscheduler and assign the SCHED_FIFO policy with a priority of 50. Then call spin on the object detecting executor.

After creating this thread, call spin on the data logging executor

# Step 3: Add tracing commands

// TODO

# Step 4: Build code

A build script is provided for you

    ./build.sh

# Step 5: Run program

// TODO: Start gazebo

Run the three nodes by entering the 3 commands below in different terminals

    ros2 run camera_demo publisher_node
    ros2 run camera_demo sensor_node
    ros2 run camera_demo actuation_node