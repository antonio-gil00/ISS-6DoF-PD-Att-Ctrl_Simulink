Welcome to the ASE372K: Attitude Dynamics Final Project.

This folder contains all the necessary files to run an simulation of the ISS earth
pointing 6-DOF dynamics on Matlab Simulink.

In this project, we aim to develop a comprehensive simulation, estimation, and control
framework for the attitude dynamics of the International Space Station (ISS). Utilizing MATLAB
and Simulink, our model incorporates the ISS's four double-gimballed Control Moment
Gyroscopes (CMGs) as primary actuators, ensuring precise attitude control. For sensing and
measurement, we simulate an Earth Horizon sensor, a gyroscope, a magnetometer, and motor
encoder sensors within each gimbal. Sensor data fusion is done using TRIAD algorithm for attitude
determinatino. Our simulation also accounts for external torque
perturbations, specifically those arising from gravity gradient and atmospheric drag, which are
crucial for realistic modeling. The ISS is considered to be in a stable circular orbit at an altitude
of 400 km, following an Earth-pointing Local Horizontal Local Vertical (LHLV) attitude profile.
This configuration ensures that the ISS maintains its bottom facing towards the Earth's center, a
critical aspect for various operational and observational purposes.

Source for Double-Gimbal CMG Steering Law:

“Steering Law for Parallel Mounted Double-Gimbaled Control Moment Gyros. Revision A -
NASA Technical Reports Server (NTRS).” NASA, NASA, ntrs.nasa.gov/citations/19810005480.
Accessed 5 Dec. 2023

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Warning: Do not edit files under src or the folder structure.

System Requirements: 
MATLAB 2024b.
Aerospace Toolbox

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Group Members: Contributions

Antonio Gil: Test Files, ISS 6-DOF Dynamics, CMG Steering Law Implementation , Attitude PD Controller, Hardware Functions, Simulation Integration.
Thomas Holzman: Gravity Gradient Torque Perturbation Function
Evan Sayer: Aerodynamic Torque Perturbation Funciton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Folder Structure

data: Containts all the data analysis functions used to generate plots in report.

doc: Contains the scripts to initalize the simulation and replicate results 
found in the report.

src: Contatins all source simulink library models and matlab files to model the ISS Earth pointing attideu dynamics.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Instructions

1. Run initFinalProject.m to add all the required folders to path.
2. Go to doc/IssLVLHPointingSim
3. Run the file test01IssLVLHPointing.m to run the simulation of the ISS rotational dynamics without any perturbing torques.
4. Run the file test02IssLVLHPointing.m to run the simulation of the ISS rotational dynamics with atmospheric torque
   and gravity gradient perturbations
4. After simulation is over data visualization plots will be automatically generated.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

