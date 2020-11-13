Update on 13 Nov 2020: This repository has moved! Please check out the latest version here: https://github.com/roahmlab/arm_planning

# Autonomous Reachability-based Manipulator Trajectory Design (ARMTD)
This repo contains the code for our recent paper: https://arxiv.org/abs/2002.01591

## Setup Requirements
To run this code, you'll need MATLAB R2018b or newer.
Also, you will need [CORA_2018](https://tumcps.github.io/CORA/).

If you want to run the GPU versions of the code, you'll need IPOPT... more detailed instructions for setting up the GPU versions of the code to follow.

Also, if you want to run anything in the `armtd_benchmark` folder, you'll have to do that in Ubuntu with MoveIt! and ROS installed. Similarly, everything in the `fetch_command_files` folder is what we used to communicate with the Fetch hardware (via ROS in MATLAB, from an Ubuntu machine).

All of the ARMTD MATLAB/MEX code has a non-GPU version, but it isn't usually real-time fast. You may have to recompile the CUDA MEX files for your own system, if you're using a GPU.

## Navigating this Repo (or [start here](https://github.com/ramvasudevan/arm_planning/tree/master/simulator_files))
Here's a brief explanation of the different folders here:
1. [armtd_benchmark](https://github.com/ramvasudevan/arm_planning/tree/master/armtd_benchmark): code for comparing armtd to CHOMP through MoveIt!
2. [fetch_command_files](https://github.com/ramvasudevan/arm_planning/tree/master/fetch_command_files): code for interfacing with the Fetch through MATLAB and ROS
3. [figures](https://github.com/ramvasudevan/arm_planning/tree/master/figures): a couple scripts for generating 
4. [maps](https://github.com/ramvasudevan/arm_planning/tree/master/maps): some code for forward kinematics
5. [simulator](https://github.com/ramvasudevan/arm_planning/tree/master/simulator): a copy of [this repo](https://github.com/skousik/simulator) that we are freezing in time
6. [simulator_files](https://github.com/ramvasudevan/arm_planning/tree/master/simulator_files): **Start here!** all the code for actually running armtd is contained in this folder.

## How to Use
Make sure that you have added the full arm_planning repo to your MATLAB path, as well as the CORA_2018 toolbox.

We will update this repo, and clean it up, to be more user-friendly. For now, check out the files in [this folder](https://github.com/ramvasudevan/arm_planning/tree/master/simulator_files/examples):
`arm_planning > simulator_files > examples`

## Credits
All the code here was written by Patrick Holmes, Bohao Zhang, Shreyas Kousik, Daphna Raz, and Corina Barbalata.
