![GitHub issues open](https://img.shields.io/github/issues/kuralme/turtlebot4_simulation)
![GitHub forks](https://img.shields.io/github/forks/kuralme/turtlebot4_simulation)
![GitHub stars](https://img.shields.io/github/stars/kuralme/turtlebot4_simulation)
![GitHub license](https://img.shields.io/github/license/kuralme/turtlebot4_simulation)

# Turtlebot4 Simulation Docker

This project provides a containerized simulation environment for the TurtleBot4 robot in a warehouse world using ROS 2 and Gazebo Harmonic. It includes navigation using **Nav2**, localization, path planning, waypoint following, behavior trees, and visualization in RViz.

<img src="assets/depot.png" alt="Depot-tb4" width="500"/>
<!-- [![Video Label](http://img.youtube.com/vid)] -->

## Requirements

- Docker/Compose
- `xhost` (for GUI access from containers)
- Linux (amd64 or arm64, auto-detected)
- NVIDIA GPU (optional, for Gazebo acceleration)

## 🐳 Launch (Docker)

Clone the Repository to your workspace, build the docker images and start:

```bash
git clone <repo-address>
cd turtlebot4_simulation/docker
make
```
