<p align="center">
<img src="logo.png" width="300">
</p>
<p align="center">
    <em>This image was generated with the assistance of AI.</em>
</p>

This repository lists FREE resources available ONLINE on the topic of guidance navigation and control for Moon landers, orbiters and rovers.

# Table of contents
* [Overview](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Overview)
  * [Spacecraft GNC](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Spacecraft-GNC)
    * [Missions](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Missions)
    * [Orbital](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Orbital)
      * [Mission Planning](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Mission-Planning)
      * [Guidance](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Guidance)
        * [Simulation](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Simulation)      
      * [Navigation](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Navigation)
        * [Sensors](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Sensors)
      * [Control](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Control)
        * [Actuators](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Actuators)
    * [Descent And Landing](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Descent-And-Landing)
      * [Guidance](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Guidance)
        * [Simulations](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Sensors)
        * [Planetary surface models](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Sensors)
      * [Navigation](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Navigation)
        * [Sensors](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Sensors)
      * [Control](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Control)
        * [Actuators](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Actuators)
  * [Lunar Rover GNC](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Lunar-Rover-GNC)
      * [Guidance](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Guidance)
        * [Simulations](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Sensors)
        * [Planetary surface models](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Sensors)
      * [Navigation](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Navigation)
        * [Sensors](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Sensors)
      * [Control](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Control)
        * [Actuators](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Actuators)

* [Courses](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#courses)
* [Podcasts](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#podcasts)
* [Newsletters](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#newsletters)
* [Online communities](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#online-communities)
* [Books](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#books)
* [Jobs](https://github.com/robmarkcole/satellite-image-deep-learning#jobs)
* [Movers and shakers on Github](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#movers-and-shakers-on-github)
* [Companies & organisations on Github](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#companies--organisations-on-github)

# Overview
Lunar guidance, navigation, and control (GN&C) refers to the systems and processes that are used to guide, navigate, and control spacecraft and other vehicles as they travel to, land on, operate on and return back from the Moon. These systems include sensors, computers, and other hardware and software that are used to determine the vehicle's position and orientation, as well as to control its motion and trajectory.

Lunar GN&C systems are typically used to:
* Determine the vehicle's position and orientation relative to the Moon's surface
* Control the vehicle's attitude and orientation (i.e., its orientation in space) and control the vehicle's trajectory (i.e., its path through space)
* Control the vehicle's propulsion system (e.g., its thrusters or engines)

This repository compiles useful resources for lunar mission GNC: 

## Spacecraft GNC

### Missions
[List of lunar missions](https://en.wikipedia.org/wiki/List_of_missions_to_the_Moon) -> Great summary of missions compiling spacecraft, launch date,	carrier rocket,	operator,	mission type and	outcome.

### Orbital 
#### Mission Planning
* [General Mission Analysis Tool](https://sourceforge.net/projects/gmat/) -> Design high fidelity missions to the moon! | GMAT script, integrates Python and MATLAB®
* [Translunar Trajectory Design and Optimization](https://in.mathworks.com/matlabcentral/fileexchange/73600-trans-lunar-trajectory-optimization-otb-mice-version?s_tid=srchtitle) -> Design preliminary lunar missions from Earth park orbit to B-plane encounter at the moon | MATLAB®
* [Propagating Trajectories from the Earth to the Moon](https://in.mathworks.com/matlabcentral/fileexchange/43067-propagating-trajectories-from-the-earth-to-the-moon?s_tid=prof_contriblnk) -> Integrates geocentric orbital equations of motion of a trajectory from the Earth to the Moon | MATLAB®
* [A MATLAB Script for Trans-Earth TCM Trajectory Optimization](https://in.mathworks.com/matlabcentral/fileexchange/93205-a-matlab-script-for-trans-earth-tcm-trajectory-optimization?s_tid=prof_contriblnk) ->  Design and optimize an impulsive trajectory correction maneuver (TCM) during the trans-Earth phase of lunar flight | MATLAB®
* [Predicting the Evolution of Lunar Orbits](https://in.mathworks.com/matlabcentral/fileexchange/40485-a-matlab-script-for-predicting-the-evolution-of-lunar-orbits?s_tid=prof_contriblnk) -> Predicts the long-term evolution of lunar orbits | MATLAB®
* [Parametric Analysis of TLI Delta-V Lunar Trajectories - OTB](https://in.mathworks.com/matlabcentral/fileexchange/73593-parametric-analysis-of-tli-delta-v-lunar-trajectories-otb?s_tid=prof_contriblnk) -> Solves for the minimum TLI delta-v using a two-body Lambert solution for the transfer trajectory from the Earth park orbit to the center of the moon | MATLAB®
* [Software list for planning from Orbital Index](https://github.com/orbitalindex/awesome-space#mission-design) -> Compilation of generic mission design software for space

#### Orbital Dynamics
* [Orbital Dynamics functions](https://github.com/cisprague/Spacecraft_Testbed/tree/master/Spacecraft_Testbed) -> Generate standard accelerations for mission design.

### Descent And Landing
#### Guidance
* [Apollo 11 Landing Guidance](https://github.com/chrislgarry/Apollo-11/blob/master/Luminary099/LUNAR_LANDING_GUIDANCE_EQUATIONS.agc), [Full repo](https://github.com/chrislgarry/Apollo-11), [Compile Software](https://github.com/virtualagc/virtualagc) -> Contains Guidance equations, Ignition Algorithm, Stopping Criteria, Surface-relative targeting, etc. | AGC source code
* [Surveyor Mission Landing Guidance](https://github.com/thomasantony/surveyor) -> Designed as an Orbiter addon that demonstrates the Surveyor lunar probe landing guidance | RUST
##### Simulations
* [Spaceflight simulation game in which you take control of a lunar lander at the final phases of its landing burn](https://github.com/arda-guler/miniLanding3D)
* [GFold terminal guidance](https://github.com/oyster-catcher/gfold) -> Compute fuel-optimal trajectory and generate thrust (constrain intial and final directions and magnitude) and angle (constrained) commands for a single throttleable engine spacecraft with independent attitude control. | Python

#### Navigation
* [Linear Covariance Analysis](https://github.com/openlunar/lincov) -> Linear covariance analysis tool for understanding navigation uncertainty for cislunar trajectory

### Lunar Rover GNC

## Courses

## Podcasts or Talks

* [Towards Autonomous Precision Lunar Landing](https://docs.google.com/presentation/d/e/2PACX-1vTJj2PLMM8m3H5_cqL_hTKUmryes4L8NuMojqLXx_lXmnx3Y6bgP3cNHxm2K0sKRoDLbL0gBGXb7iwg/pub?start=false&loop=false&delayms=3000#slide=id.g7d204c6de0_0_0) -> Dr A Kamath, UC Davis, Broad Guidance, Navigation and Control overview for lunar landers and hoppers

## Newsletters

## Online communities

## Books

## Jobs

## Movers and shakers on Github

## Companies & organisations on Github
