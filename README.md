# Minimum Snap and Minimum Jerk Trajectory Planning and Control

This project focuses on trajectory generation, comparison, and control for quadcopters. It involves the development of minimum jerk and minimum snap trajectories, time allocation optimization, and a PD controller (V. Kumar, Robotics: Aerial robotics, in: Coursera, University of Pennsylvania) for trajectory tracking. The project demonstrates the application of differential flatness, polynomial optimization, and control theory in autonomous quadcopter systems.

## Features
- **Trajectory Generation**:
  - **Minimum Jerk Trajectory**: A 5th-degree polynomial trajectory with 3rd-derivative cost minimization.
  - **Minimum Snap Trajectory**: A 7th-degree polynomial trajectory with 4th-derivative cost minimization.
- **Time Allocation**: Optimizes the time between nodes to minimize total flight time and avoid overshooting.
- **PD Controller**: Tracks the desired trajectory.
- **Obstacle Avoidance**: Demonstrates trajectory planning through predefined hoops without collisions.

## Project Structure
- **Code**: Contains MATLAB scripts for trajectory generation, time allocation, and PD control.
- **Documents**: Includes the project report (Project Report.pdf) with detailed explanations and results.