# Step 1: Edit code

In main.cc, create an executor and add the two nodes to it. Then call spin() on the executor. There is a comment where you should do this.

# Step 2: Build code

To build your workspace, run the following command from the exercise3-1 directory:

    colcon build

# Step 3: Run program

Run the three nodes with the provided run script

    ./run.sh

# Step 4: Check results

Stop the program with Ctrl+C. Find the exercise3-1.paretto file and upload it to the cactus-rt tracing visualizer.

You can visualize the latency by select "Latency" from the navigation bar on the left. At the top, select "Actuation" and "EndToEndDelay" from the drop down menus. This shows a histogram (in microseconds) of the time from image publication to actuation. Any samples larger than 17,000us (17ms) are considered to be missed deadlines.

You can also look at the datalogger (one source of interference) by selecting "data_logger_callback" and "DataLogger" in the drop down menus. This is meant to take anywhere between 1-10ms. Any measurements longer than this range are due to interference from stress on the system.

Finally, you can look at the object detector by selecting "object_detector_callback" and "ObjectDetect". This is meant to take 2ms. Any measurement longer than this range is due to interference from data logger and system stress.