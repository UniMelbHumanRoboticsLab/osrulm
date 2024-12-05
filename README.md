# Upper-Limb Musculoskeletal  Study: Integrating External-Forces for In-Depth Analysis of Robotic Assistance and Motor Learning

By Sivaprasad Kunnath, Kun Chu

Supervised by Dr Vincent Crocher, Prof. Denny Oetomo, Mr Xinliang Guo


---------------------------------------------------
## Introduction

Robotic (haptic) devices allow to investigate human motor learning by applying controlled force fields to human subjects and observing their reaction. In this context, it is of interest to have an in-depth view of the subject reaction, down to the muscle level. The field of human biomechanics offers insights into methods for measuring these reaction forces when subjected to external forces. Tools merging external force measure, muscle activation (EMG signals) and kinetic information are frequently used in gait analysis but are still lacking for Upper Limb (UL) biomechanics.
 
The objective of this project is thus to integrate external forces as applied by a robotic device into a UL musculoskeletal model. The simulation pipeline allows to investigate external force fields or robotic assistance on the subject’s muscle patterns and provide an estimation of the active muscle fibre forces by giving observed kinematic file and force file as input.

---------------------------------------------------
## Items in this folder
1. The MATLAB and Python code for arm joint positon data and deweighting force gneration
2. The simulation code of Opensim API C++ file
3. The result data of interations including muscle activation data and muscle force data

## Requirements and dependencies

The project uses or depends on the following third-party software and packages:

 - Visual Studio 2017 (or 2019?)
 - cmake >3.2 
 - OpenSIM 4.5
 - The MoBL-ARMS Dynamic Upper Limb model from https://simtk.org/projects/upexdyn
 - Dedicated Python scripts to generate UL synthetic trajectories and simulated forces from https://github.com/UniMelbHumanRoboticsLab/isbulmodel-python,
 
See [installation document](./Install.md) for details.
