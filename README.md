# ATLAS: Autonomous Traffic Light And Sign System 

## 📃 Introduction

ATLAS (Autonomous Traffic Light And Sign System) is a ROS 2 framework for autonomous robot navigation with built-in traffic sign and signal recognition. It combines vision-based line following with PID control and ArUco marker detection for precise, responsive path tracking. Modular ROS 2 nodes manage perception, decision-making, and control, enabling reliable navigation in structured environments.  

## ⚙️ Installation, Build & Run

> **⚠️ Disclaimer:**  
> This project requires Pixi to be installed on your machine **before** proceeding.  
> If you don’t have it yet, install it following the istructions here: https://pixi.sh/latest/advanced/installation/

After cloning the repository, run the following commands:

```
cd atlas
pixi install
pixi shell
colcon build --symlink-install
```

From a different terminal in the root of the `atlas` directory you can open CoppeliaSim:

```
pixi run coppelia
```

From the `scenes` folder, upload the `Circuit_final.ttt` scene using the top-left menu in CoppeliaSim, then press the `Start/resume simulation` button in coppelia.

Now open a new terminal in the root of the `atlas` directory and you are ready to start the ATLAS robot!

```
pixi shell
source install/setup.zsh
ros2 launch atlas controllers.launch
```
