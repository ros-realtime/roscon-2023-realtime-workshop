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