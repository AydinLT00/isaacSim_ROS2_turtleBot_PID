# isaacSim_ROS2_turtleBot_PID
General implementation of position control of a turtleBot (Burger) in NVIDIA IsaacSim environment controlled via ROS2 Jazzy using PID controller.. 


# 🤖 ROS 2 & Isaac Sim: Personal Learning Log

This repository serves as a personal log of my journey learning **ROS 2 Jazzy** running on **Ubuntu WSL2**, integrated with **NVIDIA Isaac Sim 5.1.0** running natively on **Windows 11**. 

The goal of this project is to bypass standard "toy" examples and build a solid understanding of robotics software architecture. Specifically, we load a raw 3D model of a robot, manually configure its Odometry (sensors) and Differential Drive (actuators) via OmniGraph, and control it autonomously to a target coordinate using a custom-built, full **PID Controller**.

---

## 🛠️ Prerequisites & Environment Setup

1. **Windows 11 Setup:**
   * NVIDIA Isaac Sim installed.
2. **WSL2 (Ubuntu) Setup:**
   * ROS 2 Jazzy installed.
   * Open your WSL2 terminal and source your ROS 2 environment:
     ```bash
     source /opt/ros/jazzy/setup.bash
     ```

---

## 🚀 Step-by-Step Implementation

### Step 1: Starting the Software
1. **Launch Isaac Sim:** Open a PowerShell terminal in Windows and run the selector script:
   ```powershell
   C:\isaacsim\isaac-sim.selector.bat
   ```
2. **Enable ROS 2 Bridge:** Once Isaac Sim is open, go to the top menu bar: `Window -> Extensions`. Search for `ros2` and enable the **ROS 2 Bridge** (ensure it's set to Autoload).

### Step 2: Preparing the Environment
1. Load a basic ground plane: `Create -> Physics -> Ground Plane` (or load your preferred grid environment).
2. Load the raw robot asset: Drag and drop the `turtlebot.usd` into the stage.
   * *Note:* At this stage, this `.usd` file is strictly a 3D visual and physical model. It has no Action Graphs (no software hooks for ROS 2 to talk to). We have to build them manually.

### Step 3: Configuring Odometry
We need the robot to broadcast its $(X, Y, \theta)$ position to ROS 2.
1. Go to the top menu: `Tools -> Robotics -> ROS2 Omnigraph -> Odometry publisher`.
2. In the properties of this new graph, set the **Chassis Frame ID** to `base_footprint`.
3. **Verify the Connection:**
   * Press **Play** in Isaac Sim to start the physics simulation.
   * Open your WSL2 terminal and run:
     ```bash
     ros2 topic list
     ```
   * You should see `/odom` in the list. *Troubleshooting: If it doesn't appear, open the Odometry graph and verify that `ChildFrameId` and the input topic are correctly set to `odom`.*

4. **Verify in RViz2:**
   * In WSL2, run `ros2 run rviz2 rviz2`.
   * Change the **Fixed Frame** to `odom`.
   * Add an Odometry display and subscribe to `/odom`. You should see the red arrow dynamically appearing, tracking the robot's state.

### Step 4: Configuring Differential Drive
We need a way for ROS 2 to send velocity commands (`/cmd_vel`) to the physics engine to spin the wheels. I built an Action Graph based on the [official NVIDIA ROS 2 Drive TurtleBot Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html).

1. Create a new Action Graph (e.g., `cmdVel`).
2. Add the required nodes to map the ROS 2 `Twist` message to the physics joints.
3. **Joint Mapping:** 
   * Use two `Constant Token` nodes containing the exact names of the robot's joints: `wheel_left_joint` and `wheel_right_joint`.
   * Feed these two tokens into a `Make Array` node.
4. **Articulation Controller:**
   * Link the array to the `Joint Names` input of an `Articulation Controller` node.
   * Set the target of the Articulation Controller to the `turtlebot3_burger` prim in your stage.
5. **Differential Controller Parameters:**
   To make the physics match reality, I configured the Differential Controller node with the physical specifications of a TurtleBot3 Burger:
   * `maxAngularSpeed`: 1.0 rad/s
   * `maxLinearSpeed`: 0.22 m/s
   * `wheelDistance`: 0.16 m
   * `wheelRadius`: 0.025 m
   * 
<img width="1825" height="790" alt="cmd_vel" src="https://github.com/user-attachments/assets/2b3dfeaf-a136-43c3-8762-a9f1b567b15d" />

---

## 🧠 Step 5: The PID Controller

Instead of teleoperating the robot with a keyboard, I wrote a custom ROS 2 Python Node to act as a **Full PID Controller**. 

### How it Works:
* **Feedback Loop:** The script subscribes to `/odom`. By reading the exact timestamp from the ROS 2 message header, it calculates a mathematically accurate $\Delta t$ (dt) for discrete-time control integration.
* **Error Calculation:** It calculates the Euclidean distance error ($v$) and Heading error ($\omega$) relative to a desired $(X, Y)$ target.
* **Control Action:** It calculates Proportional, Integral (with anti-windup clamping to prevent runaway accumulation), and Derivative responses. 
* **Non-Holonomic Logic:** A programmatic rule ensures the robot turns to face the target before applying forward velocity.
* **Actuation:** The final $v$ and $\omega$ are published directly to `/cmd_vel`, driving the Isaac Sim Action Graph.

To run the controller:
```bash
python3 full_pid_ctrl.py
```

---

## 📊 Results

When the simulation runs, the PID controller elegantly drives the Turtlebot to the target coordinate, slowing down smoothly as the proportional error decreases, and settling exactly on the target with zero steady-state error.

### Simulation Demo
![Isaac Sim Turtlebot PID Demo](turtle_pid_1.gif)

### Control Performance Data
To verify the quality of the controller, the python script logs the telemetry and plots it using `matplotlib`. 
<img width="1186" height="716" alt="image" src="https://github.com/user-attachments/assets/d01955c2-0b9a-4c5f-b189-beec01f472b5" />

---
*End of log.*
