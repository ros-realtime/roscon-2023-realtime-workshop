# Step 1: Edit code

In camera_demo.cpp, create an instance of publisher_node, sensor_node, and actuator_node and add them to an executor. Then call spin() on the executor

In actuation_node.cpp, edit subscription_ to give it a HIGH priority. Then, in the main function, create an instance of actuation_node and pass it to rclcpp::spin

In sensor_node.cpp, there are two callbacks. Since each subscription has a different importance, they are assigned different priorities. By default, a subscription is set the low priority. Leave the logger subscription at low priority, but modify the SubscriberNode constructor to give the object detection subscription HIGH priority.

Finally, edit publisher_node.cpp to give that subscription a HIGH priority.

# Step 2: Build code

A build script is provided for you

    ./build.sh

# Step 3: Run program

Run the three nodes by entering the command below

    ros2 run camera_demo camera_demo