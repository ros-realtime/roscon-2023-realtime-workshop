# Step 1: Setup

Open this workspace in a fresh terminal and run the following command to set up any dependencies

    ./setup.sh

# Step 2: Edit code

In publisher_node.cpp, create an instance of publisher node and pass it to rclcpp::spin

// TODO: Do this one by default and let the audience modify sensor_node?
In actuation_node.cpp, edit subscription_ to give it a MEDIUM priority. Then, in the main function, create an instance of actuation_node and pass it to rclcpp::spin

In sensor_node.cpp, there are two callbacks, so we need to manually create a multi-threaded executor Since each subscription has a different importance, they are assigned different priorities. By default, a subscription is set the low priority. Leave the logger subscription at low priority, but modify the SubscriberNode constructor to give the object detection subscription HIGH priority.

Still in sensor_node.cpp, modify the main function to create a SensorNode and a multithreaded executor. Put the node in the executor and call spin() on the executor.

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