# Autonomous Vehicle Project

Welcome to the Autonomous Vehicle project repository! This project focuses on developing an autonomous vehicle capable of navigating its environment, avoiding obstacles, and executing various modes of operation. Below is an overview of the project's hardware, safety features, modes of operation, control algorithms, PID calibration, potential usages, and instructions for getting started.

## Hardware Requirements

- Master Microcontroller: Raspberry Pi Pico W
  Functions as the central control unit, orchestrating all components and executing programmed tasks with efficiency and precision.
- Dual-Camera Setup
  Consists of two high-resolution cameras strategically positioned for comprehensive surveillance and object detection capabilities.
  Empowered by the Kendryte K210 AI chip for real-time processing of computer vision algorithms, enhancing the robot's perception and decision-making abilities.
- Mecanum Wheels
  Four specialized wheels arranged in a configuration that enables omnidirectional movement, allowing the robot to move smoothly and precisely in any direction.
  Provides unparalleled agility and maneuverability, essential for navigating complex environments and executing intricate tasks.
- Power System
  Lead-Acid 12V Battery: Provides reliable power supply to the robot, ensuring long-lasting operation and mobility in various environments.
  BO Motors (200 rpm): Drives the movement of the mecanum wheels, offering efficient motor control and precise maneuvering capabilities.
- HuskyLens
  Features a built-in camera and Kendryte K210 microcontroller, facilitating real-time learning and adaptation to new objects and environments.
  Enhances the robot's versatility and intelligence by dynamically expanding its capabilities based on real-world interactions.
- Motor Driver Module: DRV8833
  Two-channel motor driver module responsible for controlling the movement of the robot's mecanum wheels, ensuring smooth and efficient operation.
- Distance Sensors
  HC-SR04 Ultrasonic Sensor: Positioned at the front of the robot, accurately measures distances to objects, enabling obstacle detection and collision avoidance.
  VL53L0X LiDAR Sensor: Integrated at the rear of the robot, provides precise distance measurements for enhanced spatial awareness and navigation capabilities.
- Buck Converter: LM2596-Based
  Efficiently regulates voltage levels, ensuring stable power supply to the components and optimizing energy consumption for prolonged operation.
- Safety Switch and Control Interface
  Incorporates a safety switch for manual intervention and mode selection, allowing users to switch between autonomous and manual control modes effortlessly.
  Features a user-friendly interface, including HTML/CSS-based controls for intuitive interaction with the robot's functionalities over WiFi.

This comprehensive hardware setup equips the robot with advanced capabilities for perception, movement, intelligence, and power management, making it a versatile and powerful platform for various robotics applications.

## Key Features

- Efficient Surveillance: The dual-camera setup enables efficient surveillance, tracking, and monitoring of objects in real time, enhancing security and situational awareness.
- Real-Time Learning: With the HuskyLens's real-time learning feature, the robot dynamically adapts to new objects and environments, expanding its capabilities on the fly.
- Omnidirectional Movement: Mecanum wheels facilitate fluid and precise movement in any direction, enabling agile navigation and maneuverability in complex environments.
- Over-the-Internet Control: Remote control and monitoring capabilities via WiFi empower users to interact with the robot from anywhere, enhancing versatility and accessibility.
- Safety and Collision Avoidance: Ultrasonic and LiDAR distance sensors ensure safe navigation by providing accurate distance measurements and collision detection capabilities.
- Versatile Applications: From security and surveillance to industrial automation and healthcare assistance, the robot's versatility makes it suitable for a wide range of fields and applications.

## Potential Applications

- Security and Surveillance: Ideal for patrolling, monitoring, and securing facilities, homes, and public spaces.
- Industrial Automation: Streamlines workflows, automates tasks, and improves efficiency in industrial settings.
- Healthcare Assistance: Assists healthcare professionals, patients, and caregivers in hospitals, clinics, and assisted living facilities.
- Agriculture and Environmental Monitoring: Contributes to precision agriculture and environmental monitoring efforts.
- Education and Research: Inspires innovation and discovery in robotics, computer vision, and AI research in academic institutions and laboratories.
- Retail and Customer Service: Enhances customer service and shopping experiences in retail environments.
- Search and Rescue Operations: Aids search and rescue teams in exploring hazardous environments and detecting survivors.
- Entertainment and Hospitality: Provides interactive experiences and personalized services in entertainment venues, hotels, and theme parks.
- Military and Defense: Supports reconnaissance, surveillance, and perimeter security operations in military applications.
- Smart Home Integration: Integrates with smart home systems to automate tasks and monitor home security.

## Modes of Operation

### Autonomous Mode (Mode 1)

- Fully autonomous operation, leveraging computer vision and decision-making algorithms.
- Capable of following lines, tracking human faces, and recognizing taught objects.
- Navigates independently without human intervention, performing tasks based on detected objects.

### Manual Mode (Mode 2)

- Remote control via smartphone or computer over WiFi.
- Users can control movement, camera angles, and other functions in real time.
- Enables precise control and maneuvering for specific tasks or environments.

### Mixed Mode (Mode 3)

- Combines elements of autonomous and manual control.
- Performs programmed tasks autonomously when known objects are detected.
- Pauses and awaits manual commands in the absence of detected objects or in unfamiliar situations.

### Idle Mode (Mode 0)

- Standby

## Contributing

Contributions to this project are welcome! If you have any suggestions, improvements, or feature requests, please feel free to submit a pull request or open an issue on GitHub.

## License

This project is licensed under the [MIT License](LICENSE).

## Images:
![20240416_163559](https://github.com/chinmaykrishnroy/Autonomous_Mecanum_Robot/assets/65699140/70adbb1f-93f1-4be0-a8ad-7d696c1b8ccb)
![20240416_163405](https://github.com/chinmaykrishnroy/Autonomous_Mecanum_Robot/assets/65699140/7e52572d-f025-4eec-a681-ca2aa50b33ce)
![20240416_163551](https://github.com/chinmaykrishnroy/Autonomous_Mecanum_Robot/assets/65699140/7c1defc4-f598-414e-950c-edf725594a4f)
![20240416_163133](https://github.com/chinmaykrishnroy/Autonomous_Mecanum_Robot/assets/65699140/7ce9f9bf-5bf0-49df-abb6-6743a5493c79)
 ## Videos:
https://www.linkedin.com/posts/chinmaykrishnroy_robotics-artificialintelligence-innovation-activity-7186109686750859264-0gqj?utm_source=share&utm_medium=member_desktop
