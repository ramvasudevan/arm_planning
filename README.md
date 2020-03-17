# Autonomous Reachability-based Manipulator Trajectory Design (ARMTD)
This repo contains the code for our recent paper: https://arxiv.org/abs/2002.01591

## Setup Requirements
To run this code, you'll need MATLAB R2018b or newer. If you want to run the GPU versions of the code, you'll need IPOPT.

Also, if you want to run anything in the `armtd_benchmark` folder, you'll have to do that in Ubuntu with MoveIt! and ROS installed. Similarly, everything in the `fetch_command_files` folder is what we used to communicate with the Fetch hardware (via ROS in MATLAB, from an Ubuntu machine).

All of the ARMTD MATLAB/MEX code has a non-GPU version, but it isn't usually real-time fast. You may have to recompile the CUDA MEX files for your own system, if you're using a GPU.

## How to Use
We will update this repo, and clean it up, to be more user-friendly. For now, check out the files in [this folder](https://github.com/ramvasudevan/arm_planning/tree/master/simulator_files/examples):
`arm_planning > simulator_files > examples`

## Credits
All the code here was written by Patrick Holmes, Bohao Zhang, Shreyas Kousik, Daphna Raz, and Corina Barbalata.
