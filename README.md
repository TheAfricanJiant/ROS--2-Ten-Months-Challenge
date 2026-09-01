
---

## 🚀 **ROS 2 Ten Months Challenge**  
📅 **March 15 – December 15, 2025**  

This repository contains robotics projects and resources focused on **ROS 2** throughout a **ten-month journey**.  

---

## 📌 **Project 1: Teleoperation with micro-ROS**  

### **📜 Prerequisites**  
Ensure you have the following installed before proceeding:  
- ROS 2 (Jazzy)  
- **micro-ROS Agent** package (Install following [this guide](https://micro.ros.org/docs/tutorials/core/first_application_linux/))  
  - ⚠️ **Skip** "Building the firmware" and "Creating the micro-ROS agent"  

---

### **🛠️ Setup & Running the Project**  

#### **1️⃣ Flash the ESP Board with micro-ROS**  
Upload the **micro-ROS PlatformIO** code onto your ESP board.  

#### **2️⃣ Start the micro-ROS Agent**  
Run the micro-ROS agent to establish communication between the ESP board and ROS 2:  
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```  
🕐 **Wait** for the ESP board to connect. It will show:  
```
Subscriber created
```
✅ **Success!** Your ESP board is now communicating with ROS 2.  

---

### **🎮 Running Teleoperation**  
Open **four terminals**, sourcing ROS 2 in each one:  
```bash
source /opt/ros/jazzy/setup.bash
```

#### **📌 Terminal 1: Teleop with Keyboard**  
Run the **teleop_twist_keyboard** node:  
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
💡 **Keep this terminal active and use the control keys to move the robot.**  

#### **📌 Terminal 2: Motor Control**  
Build and run the **teleop_to_motor** package:  
```bash
ros2 run teleop_to_motor teleop_to_motor
```
🚗 **Your robot should respond to keyboard commands.**  

#### **📌 Terminal 3: Topic Echo**  
Monitor the topic used for communication between **teleop_to_motor** and the ESP board:  
```bash
ros2 topic echo /your_topic_name
```

#### **📌 Terminal 4: Visualizing the ROS Graph**  
Run **rqt_graph** to visualize node connections:  
```bash
rgt_graph
```

---

## **🛠️ Troubleshooting**  
- If the ESP board doesn’t connect, restart the micro-ROS agent and verify the board firmware.  
- If the robot doesn’t move, check if `teleop_to_motor` is running and echo the topic.  

---

Demo on  
[![Watch the video](https://www.youtube.com/watch?v=8mwOU-UvqDQ)](https://www.youtube.com/watch?v=8mwOU-UvqDQ)

