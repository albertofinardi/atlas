# ATLAS: Autonomous Traffic Light And Sign System 

## 📃 Introduction

This project implements ATLAS (Autonomous Traffic Light And Sign System), a comprehensive ROS 2–based framework for autonomous robot navigation with built-in traffic sign and signal recognition. By combining computer-vision–driven line following with PID control, ATLAS ensures accurate path tracking, while ArUco markers enable robust detection and interpretation of traffic elements. Its modular architecture consists of distributed ROS 2 nodes that communicate via topics and services, seamlessly integrating perception, decision-making, and control. We include a detailed analysis of parameter sensitivity and describe the inter-node communication patterns that underpin the system’s reliability. Experimental results in structured environments demonstrate ATLAS’s ability to follow prescribed paths consistently and to recognize and respond appropriately to traffic signs and signals, laying the groundwork for more advanced autonomous navigation applications.  

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
