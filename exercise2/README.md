`Inverted Pendulum`
===================

These packages serves as a demo inverted pendulum. The pendulum is kept upright by a PID controller.

# Basic Usage

## Build

Change directories to this folder and build:
```bash
cd exercise2
./build.sh
```

## Launch

If your computer supports graphics, you can run the following command to start the pendulum demo with RViz:

```bash
run-example2-3.sh
```

You should see something like the following:
![inverted pendulum](./imgs/invertedpendulum.png)

## Interaction

You can interact with the inverted pendulum simulation with [ROS services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).

### Reset the simulation

You can restart the pendulum simulation via the `/reset_pendulum` service. Restarting the simulation will reset the pendulum to its initial state (an initial position of 0.6 rad and inital velocity of 0.0 rad/s. ). Below is an example of how to call this service from the terminal.


```bash
ros2 service call /reset_pendulum std_srvs/srv/Empty
```

### Change the pendulum setpoint

You can change the pendulum setpoint via the `/set_desired_position` service. Below is an example of how to call this service from the terminal. This example sets the desired position for the pendulum to be 0.1 radians, where 0.0 radians is vertical. The setpoint should be between (-pi / 2, pi / 2), as the pendulum is restricted to those limits.

```bash
ros2 service call /set_desired_position inverted_pendulum_interfaces/srv/SetDesiredPosition "{desired_position: 0.1}"
```

### Change the PID constants

You can change the PID constants via the `/set_PID_constants` service. Below is an example of how to call this service from the terminal. This example sets the PID constants all to 0, effectively disabling the control loop.

```bash
ros2 service call /set_PID_constants inverted_pendulum_interfaces/srv/SetPIDConstants "{kp: 0, ki: 0, kd: 0}"
```

# Exercise 2-1: Atomics vs. Locks

This exercise covers how to pass single data between the real-time thread and the ROS thread.

In this exercise, we would like to set the pendulum position from the ROS thread. Let's look at the [main function](./src/inverted_pendulum/src/exercise2-1/main.cc) associated with this exercise. There's a `set_desired_positions_thread` that repeatedly calls `shared_context->desired_position.Set` to update the pendulum position. In this case, we update the pendulum with a sinusoid that will cause the pendulum to oscillate back and forth with a period of about 6 seconds.

The position setting logic is in [single_data.h](./src/inverted_pendulum/include/inverted_pendulum/exercise2-1/message_passing/single_data.h). In this file, we have a `Set` method that the ROS thread uses to update `value_`, and a `Get` method that the real-time thread uses to read `value_`. Thread-safety is achieved using locks.

Let's run the example. First, run `stress-ng`:

```bash
stress-ng -c $(nproc)
```

In another terminal, let's run the exercise:
```bash
run-exercise-1.sh
```

You should see RViz with the pendulum oscillating slowly. The PID controller output will be printed to the terminal every second. Loop overruns will also appear in the logs.

Stop running the program after several seconds. If you do not terminate the program, after 2 minutes, tracing will terminate to keep file sizes low.

This will generate a trace file called `exercise2-1.perfetto`. A sample result is included [here](./results/example2-1/baseline.perfetto). In the browser, open [Perfetto](https://cactusdynamics.github.io/perfetto/) to visualize the trace.

Look at the timeline and the latency histogram for the `GetDesiredPosition` slice.

![histogram of latency from baseline exercise 2-1 with locks](./imgs/exercise2-1baselinehistogram.png)
![timeline slice showing largest latency from baseline exercise 2-1 with locks](./imgs/exercise2-1baselinetimeline.png)

Now, let's try using an atomic instead of a lock. Replace the `double` with a `std::atomic<double>`. When using an atomic, be sure to check that the atomic is lock free with `static_assert(std::atomic<double>::is_always_lock_free)`. 

The solution can be found [here](./solutions/single_data.h). Below are the example results for the solution using atomics. The associated example trace file is [here](./results/example2-1/solution.perfetto).

![histogram of latency from exercise 2-1 solution with atomics](./imgs/exercise2-1baselinehistogram.png)
![timeline slice showing largest latency from exercise 2-1 solution with atomics](./imgs/exercise2-1baselinetimeline.png)


# Exercise 2-2

First, run the example:

```bash
run-example2-2.sh
```

# Exercise 2-3

First, run the example:

```bash
run-example2-3.sh
```

# Solutions and Results
Example trace files can be found in the [results folder](./results/). For all results, `stress-ng` was used to stress the CPUs. These results files were generated on a Raspberry Pi 4 with a real-time kernel. This image can be found [here](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.3_v5.15.98-rt62-raspi_ros2_humble).

Solutions for all exercises can be found in the [solutions folder](./solutions/). The content of these files can be copy-pasted into the header files corresponding to each exercise to reproduce the solutions.
