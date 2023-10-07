# Step 1: Edit code

In main.cc, copy your existing changes from exercise3-1. But change the executor to a MultiThreadedExecutor this time.

To make use of the modified real-time executor, subscriptions have to specify their priority. CameraProcessingNode has two subscriptions: object detector and data logger. The former is more important. Set it accordingly in application_nodes.cc file. There is a comment in the constructor where you should do this.

ActuationNode has already been assigned high priority. Examine it for sample code.

# Step 2: Build code

To build your workspace, run the following command from the exercise3-1 directory:

    colcon build

# Step 3: Run program

Run the three nodes with the provided run script

    ./run.sh

# Step 4: Check results

Stop the program with Ctrl+C. Find the exercise3-1.paretto file and upload it to the cactus-rt tracing visualizer.

You can visualize the latency by select "Latency" from the navigation bar on the left. At the top, select "Actuation" and "EndToEndDelay" from the drop down menus. This shows a histogram (in microseconds) of the time from image publication to actuation. Any samples larger than 17,000us (17ms) are considered to be missed deadlines. There should be none. DataLogger and ObjectDetector can both run in under this time window if uninterrupted.

You can look at the datalogger by selecting "data_logger_callback" and "DataLogger" in the drop down menus. This is meant to take anywhere between 1-10ms. All measurments should be in this range (exceeding by a few microseconds is normal)

Finally, you can look at the object detector by selecting "object_detector_callback" and "ObjectDetect". This is meant to take 2ms. Since this is the highest priority task on the system, all measurements will be less than 2ms.