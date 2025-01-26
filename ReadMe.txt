Welcome to the ASE372K: Attitude Dynamics Final Project.

This folder contains all the necessary files to run an simulation of the ISS earth
pointing  6-DOF dynamcis. For more details please refer to the report.

Warning: Do not edit files under src or the folder structure.

System Requirements: 
MATLAB 2024b.
Aerospace Toolbox

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Group Members: Contributions

Antonio Gil: Test Files, ISS 6-DOF Dynamics, CMG Steering, Attitude PD Controller, Hardware Functions, Simulation Integration.
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

