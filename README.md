## Open source files for the paper titled: [**"Sensor Observability Analysis for Maximizing Task-Space Observability of Articulated Robots"**](https://www.researchgate.net/profile/Christopher-Yee-Wong/research) 
**Status:** Manuscript currently under review

This repository contains simulation files for a custom 3-DOF planar robot using the structure shown below to showcase the application and utility of the proposed method *Sensor Observability Analysis*. 
Various redundancy resolution strategies can be selected to observe the different effects of each strategy. 
The redundancy resolution strategies include:
* minimize joint motion
* maximize kinematic manipulability index
* maximize sensor observability index
* maximize sensor observability in the *x*- or *y*-axis

<img src="/3DOFcustombot_structure.png" width="400">

# File Struture:
* `README.md` file: This readme file
* `Custom3DOFRobotSimulation` Folder: Contains all simulation files and additional functions to support main file
    * `main.m`: Main smulation file. Flags at the beginning of the file are used to select between the different strategies and also to choose whether to display the robot animation or not. 
    * `calc3DOFdwdq.m`: Function to calculate the partial derivatives for kinematic manipulability index.
    * `calc3DOFdodq.m`: Function to calculate the partial derivatives for sensor observability index.
    * `calc3DOFdodq1xis.m`: Function to calculate the partial derivatives for sensor observability index in a single axis. 

# Installation:
Requires Matlab with Robotics Toolbox and Image Processing Toolbox (tested with Matlab R2021a)

# Contributors/Authors:
* Christopher Yee WONG (christopher.wong2 [at] usherbrooke.ca)
* Wael SULEIMAN (wael.suleiman [at] usherbrooke.ca)

<a href="https://www.usherbrooke.ca/"><img src="/UdeS_logo.png" width="300"></a>
