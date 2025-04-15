# Awesome Open Lunar Guidance, Navigation and Control Resources

<p align="center">
<img src="logo.png" width="300">
</p>
<p align="center">
    <em>This image was generated with the assistance of AI.</em>
</p>

This repository lists FREE resources available ONLINE on the topic of guidance navigation and control for Moon landers, orbiters and rovers. Check out my motivation behind creating this page on [Medium](https://medium.com/@chandrabhraman/curating-awesome-lunar-gnc-resources-my-passion-for-lunar-guidance-navigation-and-control-dbb16b666439)

# Table of contents
* [Overview](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Overview)
  * [Spacecraft GNC](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Spacecraft-GNC)
    * [Missions](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Missions)
    * [Orbital](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Orbital)
      * [Mission Planning](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Mission-Planning)
      * [Orbital Dynamics & Astrodynamics Tools](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Orbital-Dynamics--Astrodynamics-Tools)
      * [Guidance, Navigation and Control](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Guidance-Navigation-and-Control-Orbital)
    * [Descent And Landing](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Descent-And-Landing)
      * [Guidance, Navigation and Control](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Guidance-Navigation-and-Control-Landing)
      * [Simulators & Testbeds](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Simulators--Testbeds-Landing)
      * [Terrain Relative Navigation (TRN) & Hazard Detection](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Terrain-Relative-Navigation-TRN--Hazard-Detection)
      * [Datasets & Maps (for Landing)](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Datasets--Maps-for-Landing)
  * [Lunar Rover GNC](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Lunar-Rover-GNC)
      * [Guidance (Path Planning & Traversability)](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Guidance-Path-Planning--Traversability)
        * [Simulations](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Simulations-Rover)
        * [Planetary surface models & Datasets](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Planetary-surface-models--Datasets-Rover)
      * [Navigation (Localization & Mapping)](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Navigation-Localization--Mapping)
        * [Sensors & Sensor Fusion](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Sensors--Sensor-Fusion)
        * [Visual Odometry & SLAM](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Visual-Odometry--SLAM)
      * [Control](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Control-Rover)
        * [Actuators & Locomotion](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Actuators--Locomotion)
* [Core Libraries & Frameworks](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Core-Libraries--Frameworks)
* [Courses](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#courses)
* [Chandrabhraman resources](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#chandrabhraman-resources)
* [Podcasts or Talks](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#podcasts-or-talks)
* [Newsletters](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#newsletters)
* [Online communities](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#online-communities)
* [Books & Papers](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#books--papers)
* [Jobs](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#jobs)
* [Movers and shakers on Github](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#movers-and-shakers-on-github)
* [Companies & organisations on Github](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#companies--organisations-on-github)

# Overview
Lunar guidance, navigation, and control (GN&C) refers to the systems and processes that are used to guide, navigate, and control spacecraft and other vehicles as they travel to, land on, operate on and return back from the Moon. These systems include sensors, computers, and other hardware and software that are used to determine the vehicle's position and orientation, as well as to control its motion and trajectory.

Lunar GN&C systems are typically used to:
* Determine the vehicle's position and orientation relative to the Moon's surface or other celestial bodies.
* Control the vehicle's attitude and orientation (i.e., its orientation in space).
* Control the vehicle's trajectory (i.e., its path through space).
* Control the vehicle's propulsion system (e.g., its thrusters or engines) and other actuators.
* Enable autonomous operations, including hazard avoidance and precise maneuvering.

This repository compiles useful resources for lunar mission GNC:

## Spacecraft GNC

### Missions
* [List of lunar missions](https://en.wikipedia.org/wiki/List_of_missions_to_the_Moon) -> Great summary of missions compiling spacecraft, launch date,	carrier rocket,	operator,	mission type and	outcome.
* [Fun animation of the Luna 9 lander](https://www.youtube.com/watch?v=pDdWbrwsFTM) -> Covers the complex mission and landing sequence of the Luna 9 mission.
* [NASA PDS Lunar Data Node](https://pds-lunar.jpl.nasa.gov/) -> Access point for data from numerous past and present lunar missions (LRO, Apollo, Clementine, LCROSS, etc.). Crucial for GNC development and validation.
* [Apollo Lunar Surface Journal](https://www.hq.nasa.gov/alsj/) -> Detailed transcripts, commentary, images, and video from the Apollo lunar surface missions, including GNC operations discussions.
* [Artemis Program](https://www.nasa.gov/specials/artemis/) -> Official NASA site for the ongoing Artemis program, including mission updates, objectives, and related technology development (including GNC).
* [ISRO Chandrayaan-3 Mission Page](https://www.isro.gov.in/Chandrayaan3.html) -> Official Indian Space Research Organisation page for Chandrayaan-3, includes mission objectives, updates, and image galleries related to the successful landing.
* [CNSA Lunar Exploration Program (CLEP)](http://www.cnsa.gov.cn/english/index.html) -> Official China National Space Administration site (English version). Look for mission descriptions and press releases on Chang'e missions (though detailed technical GNC info is limited).

### Orbital
#### Mission Planning
* [General Mission Analysis Tool (GMAT)](https://sourceforge.net/projects/gmat/) -> Design high fidelity missions to the moon! NASA open-source. | GMAT script, integrates Python and MATLAB®
* [Translunar Trajectory Design and Optimization](https://in.mathworks.com/matlabcentral/fileexchange/73600-trans-lunar-trajectory-optimization-otb-mice-version?s_tid=srchtitle) -> Design preliminary lunar missions from Earth park orbit to B-plane encounter at the moon | MATLAB®
* [Propagating Trajectories from the Earth to the Moon](https://in.mathworks.com/matlabcentral/fileexchange/43067-propagating-trajectories-from-the-earth-to-the-moon?s_tid=prof_contriblnk) -> Integrates geocentric orbital equations of motion of a trajectory from the Earth to the Moon | MATLAB®
* [A MATLAB Script for Trans-Earth TCM Trajectory Optimization](https://in.mathworks.com/matlabcentral/fileexchange/93205-a-matlab-script-for-trans-earth-tcm-trajectory-optimization?s_tid=prof_contriblnk) ->  Design and optimize an impulsive trajectory correction maneuver (TCM) during the trans-Earth phase of lunar flight | MATLAB®
* [Predicting the Evolution of Lunar Orbits](https://in.mathworks.com/matlabcentral/fileexchange/40485-a-matlab-script-for-predicting-the-evolution-of-lunar-orbits?s_tid=prof_contriblnk) -> Predicts the long-term evolution of lunar orbits | MATLAB®
* [Parametric Analysis of TLI Delta-V Lunar Trajectories - OTB](https://in.mathworks.com/matlabcentral/fileexchange/73593-parametric-analysis-of-tli-delta-v-lunar-trajectories-otb?s_tid=prof_contriblnk) -> Solves for the minimum TLI delta-v using a two-body Lambert solution for the transfer trajectory from the Earth park orbit to the center of the moon | MATLAB®
* [Software list for planning from Orbital Index](https://github.com/orbitalindex/awesome-space#mission-design) -> Compilation of generic mission design software for space.
* [Nyx](https://gitlab.com/chrisrabotin/nyx) -> High-fidelity interplanetary trajectory design and GNC simulation framework. | Rust
* [PorkchopPy](https://github.com/poliastro/porkchoppy) -> Interactive porkchop plots for interplanetary mission design using poliastro. | Python

#### Orbital Dynamics & Astrodynamics Tools
* [Orbital Dynamics functions](https://github.com/cisprague/Spacecraft_Testbed/tree/master/Spacecraft_Testbed) -> Generate standard accelerations for mission design. | MATLAB®
* [Poliastro](https://github.com/poliastro/poliastro) -> Open-source Python library for Astrodynamics and Orbital Mechanics. Supports various propagators and maneuvers. | Python
* [Orekit](https://www.orekit.org/) -> Low-level space dynamics library, robust, widely used in industry and academia. | Java (Python wrapper available)
* [SPICE Toolkit](https://naif.jpl.nasa.gov/naif/toolkit.html) -> Essential NASA/NAIF toolkit for handling space geometry, time, and ephemeris data (kernels). Wrappers available for many languages. | C, Fortran (wrappers for Python, MATLAB, IDL, Java)
* [SpiceyPy](https://github.com/AndrewAnnex/SpiceyPy) -> Python wrapper for the NASA NAIF SPICE Toolkit. | Python
* [Astropy](https://www.astropy.org/) -> Core Python package for astronomy, includes coordinate transformations, time handling, and constants relevant to space science. | Python
* [jplephem](https://github.com/brandon-rhodes/python-jplephem) -> Python library to load and use JPL ephemerides for planetary positions. | Python

#### Guidance, Navigation and Control (Orbital) <a name="Guidance-Navigation-and-Control-Orbital"></a>
* [Basilisk Astrodynamics Simulation Framework](https://github.com/AVSLab/basilisk) -> High-fidelity, modular simulation framework for GNC algorithm development and testing. Includes orbital and attitude dynamics, sensor/actuator models. | Python, C++
* [NASA Core Flight System (cFS)](https://github.com/nasa/cFS) -> Platform and project independent reusable flight software framework developed by NASA Goddard. Includes GNC applications and libraries. | C
* [Monte](https://montepy.jpl.nasa.gov/) -> NASA/JPL's state-of-the-art trajectory determination and mission design software (limited public access, but good to be aware of). | Python, C++
* [OpenSatKit](https://github.com/OpenSatKit/OpenSatKit) -> Open source kit for satellite ground system development, based on COSMOS, includes tools relevant to GNC testing. | Ruby, C/C++

### Descent And Landing
#### Guidance, Navigation and Control (Landing) <a name="Guidance-Navigation-and-Control-Landing"></a>
* [Apollo 11 Landing Guidance](https://github.com/chrislgarry/Apollo-11/blob/master/Luminary099/LUNAR_LANDING_GUIDANCE_EQUATIONS.agc), [Full repo](https://github.com/chrislgarry/Apollo-11), [Compile Software](https://github.com/virtualagc/virtualagc) -> Contains Guidance equations, Ignition Algorithm, Stopping Criteria, Surface-relative targeting, etc. | AGC source code
* [Surveyor Mission Landing Guidance](https://github.com/thomasantony/surveyor) -> Designed as an Orbiter addon that demonstrates the Surveyor lunar probe landing guidance | RUST
* [GFold terminal guidance](https://github.com/oyster-catcher/gfold) -> Compute fuel-optimal trajectory and generate thrust (constrain intial and final directions and magnitude) and angle (constrained) commands for a single throttleable engine spacecraft with independent attitude control. | Python
* [Linear Covariance Analysis](https://github.com/openlunar/lincov) -> Linear covariance analysis tool for understanding navigation uncertainty for cislunar trajectory | Python
* [Lunar Descent Fuel-optimized guidance](https://jckantor.github.io/CBE30338/07.02-Soft-Landing-a-Rocket.html) -> Lunar descent guidance optimization scripts using Pyomo. | Python
* [ECOS (Embedded Conic Solver)](https://github.com/embotech/ecos) -> Numerical software for solving convex second-order cone programs (SOCPs), often used in optimal control and guidance problems (like powered descent). | C
* [OSQP (Operator Splitting Quadratic Program Solver)](https://github.com/osqp/osqp) -> Solver for quadratic programs that arise in guidance and control applications. | C (Wrappers for Python, MATLAB, etc.)
* [CVXPY](https://github.com/cvxpy/cvxpy) -> Python-embedded modeling language for convex optimization problems. Useful for prototyping GNC algorithms. | Python
* [CasADi](https://github.com/casadi/casadi) -> Symbolic framework for numeric optimization, implementing automatic differentiation. Useful for trajectory optimization and optimal control. | Python, C++, MATLAB

#### Simulators & Testbeds (Landing) <a name="Simulators--Testbeds-Landing"></a>
* [Spaceflight simulation game in which you take control of a lunar lander at the final phases of its landing burn](https://github.com/arda-guler/miniLanding3D) -> Simple game demonstrating landing dynamics. | C++ (likely, based on typical game dev)
* [Basilisk Lander Module](https://basilisk-sim.readthedocs.io/en/latest/Tutorials/tutorial_LunarLander.html) -> Specific tutorial and modules within Basilisk for simulating powered lunar descent GNC. | Python, C++
* [Habitat reconfiguration dynamics and control testbed](https://ntrs.nasa.gov/citations/20170001195) -> Paper discussing a physical testbed, concepts might be adaptable. [Open Access via NTRS]
* [Kerbal Space Program (with realism mods)](https://www.kerbalspaceprogram.com/) -> While a game, mods like Principia (N-body gravity) and Realism Overhaul make it a surprisingly useful (and challenging) educational tool for understanding orbital mechanics and landing GNC concepts.

#### Terrain Relative Navigation (TRN) & Hazard Detection <a name="Terrain-Relative-Navigation-TRN--Hazard-Detection"></a>
* [FederNet](https://github.com/UninaLabs-EO/FederNet) -> CNN based autonomous terrain-relative navigation for lunar landing | Python
* [OpenCV (Open Source Computer Vision Library)](https://opencv.org/) -> Foundational library for image processing and computer vision tasks essential for TRN (feature detection, matching, optical flow). | C++, Python, Java
* [PCL (Point Cloud Library)](https://pointclouds.org/) -> Library for processing 2D/3D image and point cloud data, relevant for LiDAR-based TRN and hazard mapping. | C++
* [Stereo Pipeline](https://github.com/NeoGeographyToolkit/StereoPipeline) -> NASA Ames toolkit for creating DEMs and orthoimages from stereo pairs, useful for generating base maps for TRN. | C++
* [Ames Stereo Pipeline Examples](https://stereopipeline.readthedocs.io/en/latest/examples.html) -> Examples include processing LRO NAC images.
* [PLIA (Planetary Lunar Image Analysis) tool](https://github.com/esa/PLIA) -> ESA tool for analysing planetary images, potentially useful for map generation or feature identification. | Python

#### Datasets & Maps (for Landing) <a name="Datasets--Maps-for-Landing"></a>
* [LRO LOLA DEMs](https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/lrolol_1xxx/data/) -> High-resolution Digital Elevation Models from the Lunar Reconnaissance Orbiter Laser Altimeter. Essential for TRN map matching and hazard detection simulation. [Open Access via NASA PDS]
* [LRO LROC NAC Images](https://pds-imaging.jpl.nasa.gov/volumes/lro.html) -> High-resolution Narrow Angle Camera images from LRO, used for visual navigation, base map creation, and hazard assessment. [Open Access via NASA PDS]
* [USGS Unified Moon Map](https://www.usgs.gov/centers/astrogeology-science-center/science/moon-geologic-map) -> Various geological and topographic maps of the Moon. [Open Access via USGS]
* [Moon Trek](https://trek.nasa.gov/moon/) -> NASA web portal for accessing and visualizing lunar data layers (imagery, elevation, slopes, etc.). Can download data. [Open Access Portal]
* [QuickMap](https://quickmap.lroc.asu.edu/) -> LROC tool for browsing and downloading LRO NAC and WAC images and LOLA data. [Open Access Portal]
* [SLDEM2015: Global lunar DEM (118m) and derived products](https://astrogeology.usgs.gov/search/map/Moon/LRO/LOLA/SLDEM2015_118m_ClrShade) -> Derived from LOLA data, widely used basemap. [Open Access via USGS]

### Lunar Rover GNC

#### Guidance (Path Planning & Traversability) <a name="Guidance-Path-Planning--Traversability"></a>
* [ROS Navigation Stack](http://wiki.ros.org/navigation) -> Provides algorithms like global planners (A*, D* Lite variations) and local planners (DWA, TEB) adaptable for rover path planning. | C++, Python (within ROS)
* [OMPL (Open Motion Planning Library)](https://ompl.kavrakilab.org/) -> State-of-the-art sampling-based motion planning algorithms. Integrates with ROS. | C++
* [PyRoboPlan](https://github.com/aescande/pyroboplan) -> Python based motion planning library including RRT, RRT*. | Python
* [AI Planning Tools (PDDL based)](https://github.com/potassco/clingo) -> For higher-level task planning which informs guidance objectives. Answer Set Programming tools like Clingo are relevant. | Python interfaces exist
* [Slope and Hazard Maps (derived from DEMs)](https://trek.nasa.gov/moon/) -> Use tools like Moon Trek or GDAL to derive slope, roughness, and identify hazards from LOLA DEMs for traversability analysis. [Open Access Data]

#### Simulations (Rover) <a name="Simulations-Rover"></a>
* [Gazebo Simulator](http://gazebosim.org/) -> Widely used robotics simulator with ROS integration. Can model rover dynamics, sensors (cameras, IMUs, LiDAR), and lunar terrain. | C++
* [Ignition Gazebo](https://ignitionrobotics.org/) -> Successor to Gazebo, modular architecture. | C++
* [Webots](https://cyberbotics.com/) -> Open-source robot simulator, supports ROS, can model planetary environments. | C++, Python, MATLAB
* [PyBullet](https://pybullet.org/wordpress/) -> Python module for physics simulation, robotics, and ML. Can be used for simpler rover dynamics simulations. | Python
* [Chrono::Vehicle](https://github.com/projectchrono/chrono-vehicle) -> Part of Project Chrono physics engine, specialized for vehicle dynamics including tracked/wheeled vehicles on deformable terrain. | C++

#### Planetary surface models & Datasets (Rover) <a name="Planetary-surface-models--Datasets-Rover"></a>
* [LOLA DEMs and LROC NAC Images](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Datasets--Maps-for-Landing) -> As listed above, essential for creating realistic rover simulation environments and navigation base maps. [Open Access Data]
* [USGS Astrogeology Science Center](https://www.usgs.gov/centers/astrogeology-science-center) -> Source for various planetary maps and DEMs, including the Moon. [Open Access Data]
* [Planetary Computer Datasets](https://planetarycomputer.microsoft.com/catalog) -> Microsoft's collection includes lunar DEMs and imagery, accessible via API. [Open Access Portal]

#### Navigation (Localization & Mapping) <a name="Navigation-Localization--Mapping"></a>
* [ROS `robot_localization`](http://wiki.ros.org/robot_localization) -> Package for fusing data from various sensors (IMU, odometry, visual odometry, GPS-like systems) using Extended Kalman Filters (EKFs) or Unscented Kalman Filters (UKFs). | C++
* [GTSAM (Georgia Tech Smoothing and Mapping)](https://gtsam.org/) -> Library implementing smoothing and mapping using factor graphs, often used in state-of-the-art SLAM systems. | C++ (Python wrapper available)
* [Star Tracker Simulation](https://github.com/AVSLab/basilisk) -> Simulators like Basilisk include star tracker models for attitude determination. | Python, C++

#### Sensors & Sensor Fusion <a name="Sensors--Sensor-Fusion"></a>
* [Sensor models in Simulators (Gazebo, Basilisk, etc.)](https://github.com/chandrabhraman/awesome-lunar-gnc-resources#Simulations-Rover) -> These simulators provide plugins/modules to model cameras, IMUs, LiDARs, sun sensors, star trackers with noise and error characteristics.
* [Kalman Filter Libraries](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) -> Implementations of various Kalman filters useful for sensor fusion. | Python
* [imu_tools (ROS)](http://wiki.ros.org/imu_tools) -> ROS package for IMU data filtering and visualization. | C++

#### Visual Odometry & SLAM <a name="Visual-Odometry--SLAM"></a>
* [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) -> State-of-the-art feature-based visual(-inertial) SLAM system. Adaptable with effort to sparse lunar environments. | C++
* [OpenVSLAM](https://github.com/xdspacelab/openvslam) -> Versatile visual SLAM framework, supports various camera models and integrates well with ROS. | C++
* [VINS-Mono / VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) -> Tightly-coupled visual-inertial odometry frameworks. | C++
* [Libviso2](https://github.com/srv/libviso2) -> Popular library for feature-based stereo/mono visual odometry. | C++
* [DSO (Direct Sparse Odometry)](https://github.com/JakobEngel/dso) -> Direct method (no features) visual odometry, potentially robust in certain texture-poor lunar areas. | C++

#### Control (Rover) <a name="Control-Rover"></a>
* [ROS Control](http://wiki.ros.org/ros_control) -> Framework in ROS for robot control, providing interfaces for hardware abstraction, controllers, and transmissions. | C++
* [MoveIt](https://moveit.ros.org/) -> Primarily for manipulation, but includes controllers and kinematics/dynamics libraries potentially useful for rover arm/instrument control. | C++
* [Control Toolbox](https://github.com/ethz-adrl/control-toolbox) -> ETH Zurich library for optimal control, estimation, and MPC, potentially applicable to rover motion control. | C++

#### Actuators & Locomotion <a name="Actuators--Locomotion"></a>
* [Gazebo Plugins](http://gazebosim.org/tutorials?tut=ros_control) -> Examples of how to model actuators (motors, joints) and interface them with `ros_control` within the Gazebo simulator.
* [Wheel-Soil Interaction Models](https://www.isprs.org/proceedings/xxxvii/congress/1b_pdf/86.pdf) -> Research area, often involves custom models or simplified physics engine approximations (e.g., tuning friction parameters in Gazebo/Bullet). Papers and specialized codebases exist but less standardized open source. [Example Open Access Paper]

## Core Libraries & Frameworks

* [Poliastro](https://github.com/poliastro/poliastro) -> Python library for Astrodynamics.
* [Orekit](https://www.orekit.org/) -> Java library for low-level space dynamics.
* [Basilisk](https://github.com/AVSLab/basilisk) -> Python/C++ framework for astrodynamics simulation.
* [SPICE Toolkit](https://naif.jpl.nasa.gov/naif/toolkit.html) -> NASA toolkit for space geometry/time data.
* [NASA cFS](https://github.com/nasa/cFS) -> NASA's reusable flight software framework.
* [ROS (Robot Operating System)](http://www.ros.org/) -> Framework for robotics software development (essential for rovers).
* [OpenCV](https://opencv.org/) -> Library for computer vision (TRN, VO, SLAM).
* [PCL (Point Cloud Library)](https://pointclouds.org/) -> Library for point cloud processing (TRN, SLAM).

## Courses

* [Attitude control for space mission](https://www.coursera.org/learn/capstone-mars-mission) -> 5 week course for developing attitude control for space missions, applicable for lunar missions as well. (Coursera course, may require payment/subscription for full access).
* [MIT OpenCourseWare - Astrodynamics](https://ocw.mit.edu/courses/16-346-astrodynamics-fall-2008/) -> Lecture notes and assignments from MIT's Astrodynamics course. [Open Access]
* [MIT OpenCourseWare - Estimation and Control of Aerospace Systems](https://ocw.mit.edu/courses/16-323-estimation-and-control-of-aerospace-systems-spring-2019/) -> Covers Kalman filtering and control theory relevant to GNC. [Open Access]
* [CU Boulder - Statistical Estimation for Aerospace Systems](https://www.coursera.org/learn/state-estimation-localization-self-driving-cars) -> Covers Kalman/Bayesian filtering with aerospace applications (though framed for self-driving cars, principles apply). (Coursera course, may require payment/subscription).
* [ESA - Basics of Spaceflight](https://www.esa.int/Education/Basics_of_spaceflight/Basics_of_spaceflight) -> Foundational knowledge from ESA. [Open Access]
* [edX Space Mission Design and Operations (DelftX)](https://www.edx.org/school/delftx) -> Search for current space-related courses and MicroMasters programs offered by TU Delft on edX. The original XSeries link is deprecated. (edX courses often have audit options but may require payment for full access/certificate).

## Chandrabhraman resources

* [Lunar Reconnaisance Orbiter GNC analysis](https://github.com/chandrabhraman/lro-analysis) -> tools for accessing the SPICE ephemeris data for LRO and looks at analysing the pointing and rate control of the spacecraft.

## Podcasts or Talks

* [Towards Autonomous Precision Lunar Landing (Slides)](https://docs.google.com/presentation/d/e/2PACX-1vTJj2PLMM8m3H5_cqL_hTKUmryes4L8NuMojqLXx_lXmnx3Y6bgP3cNHxm2K0sKRoDLbL0gBGXb7iwg/pub?start=false&loop=false&delayms=3000#slide=id.g7d204c6de0_0_0) -> Dr A Kamath, UC Davis, Broad Guidance, Navigation and Control overview for lunar landers and hoppers [Open Access Slides]
* [Programming the Moon Landing Guidance Computer | Don Eyles (Video)](https://www.youtube.com/watch?v=bsH5HbO7OPE) -> Interview with Don Eyles, the programmer of the lunar landing Apollo missions, covers the braking maneuvers, abort logic, and much more! [Open Access Video]
* [Chang'e 4 landing (Video)](https://www.youtube.com/watch?v=Hnf8-MIbuow) -> Describes the landing of Chang'e 4, landing site selection, powered descent, terminal phase described in reasonable detail. [Open Access Video]
* [Apollo 11: The Moon Landing Mission (Podcast Episode)](https://www.nasa.gov/podcasts/houston-we-have-a-podcast/apollo-11-the-mission) -> Example link to NASA's "Houston We Have a Podcast". Search NASA archives (e.g., `archive.org`, NASA site search) for more historical audio/video on Apollo GNC. [Open Access Audio/Video]
* [Apollo 13: The Triumph of Navigation and Control (Podcast Audio)](https://www.nasa.gov/sites/default/files/atoms/audio/ep139_apollo_13.mp3) -> from the "NASA InSight" podcast covers the guidance, navigation, and control systems used in the Apollo 13 mission. [Open Access Audio]
* [The Orbital Mechanics Podcast](https://theorbitalmechanics.com/) -> Covers spaceflight news and concepts, often touching on GNC topics. [Free Podcast]
* [Main Engine Cut Off (MECO) Podcast](https://mainenginecutoff.com/) -> In-depth analysis of spaceflight news, policy, and technology, including lunar missions. [Free Podcast]
* [Planetary Radio](https://www.planetary.org/planetary-radio) -> Covers space exploration broadly, features interviews with scientists and engineers involved in lunar missions. [Free Podcast]
* [Figuring Out Chandrayaan-3's Landing Algorithm | Spacecraft GNC Ep.1](https://www.youtube.com/watch?v=y3b7k47ZqLU) -> Explores the GNC algorithm potentially used for the Chandrayaan-3 lunar landing. [Open Access Video]
* [Apollo Spacecraft GNC System Premier](https://www.youtube.com/watch?v=6_313RjGzP8) -> Provides a basic overview of the Apollo Guidance, Navigation, and Control system. [Open Access Video]
* [Apollo GNC Tutorials (Example: P52 IMU Alignment)](https://www.youtube.com/watch?v=KqB6y7Z0fXg) -> Offers specific tutorials on using the Apollo GNC system (search YouTube for "Apollo GNC Tutorials Nuclear and Space Education" for more). [Open Access Video]
* [Light Years Ahead | The 1969 Apollo Guidance Computer](https://www.youtube.com/watch?v=xx7Lfh5SKUQ) -> Discusses the AGC's crucial role in guidance, navigation, and control. [Open Access Video]
* [How NASA Modeled Artemis I in MATLAB and Simulink](https://www.mathworks.com/videos/how-nasa-modeled-artemis-i-in-matlab-and-simulink-1671013335187.html) -> Explains modeling and validation of the GNC system for the Artemis I Orion spacecraft. [Free Video (MathWorks site)]
* [Robotic Lunar Lander (APL/NASA Marshall)](https://www.youtube.com/watch?v=Yh33-86LhVg) -> Discusses development of GNC for autonomous precision landings (e.g., Mighty Eagle). [Open Access Video]
* [The Intuitive Machines Lunar Lander Is About To Attempt Touchdown (IM-1 Odysseus)](https://www.youtube.com/watch?v=2jH6L4w7ZzI) -> Explains the final descent phase, role of GNC, TRN, and hazard detection for the IM-1 landing. [Open Access Video]
* [SpaceIL Beresheet Lunar Lander Telemetry Visualization](https://www.youtube.com/watch?v=QfkWSkeLZFU) -> Visualization of telemetry during the Beresheet landing attempt, highlighting GNC issues. [Open Access Video]
* [Beresheet - Final commands before landing on the Moon](https://www.youtube.com/watch?v=Qfh4adGNMAw) -> Mission control perspective during Beresheet landing attempt, referencing GNC malfunctions. [Open Access Video]
* [Analysis of Chandrayaan 2 (Vikram) landing failure](https://www.youtube.com/watch?v=kBE9z3hHz0Y) -> Analysis of telemetry data discussing potential GNC issues during Vikram's landing attempt. [Open Access Video]
* [Japan's SLIM Moon lander landing animation with GNC focus](https://www.youtube.com/watch?v=nvFdnqd-9ZI) -> Official JAXA animation showing SLIM's planned pinpoint landing sequence, highlighting image-based navigation and hazard avoidance (GNC functions). [Open Access Video]
* [Firefly Aerospace Blue Ghost Mission 1 Animation](https://www.youtube.com/watch?v=z8jMIhZE5uQ) -> Animation showing the planned mission profile for Firefly's Blue Ghost lander, including the GNC-dependent powered descent phase. [Open Access Video]

## Newsletters

* [Observe The Moon newsletter](https://moon.nasa.gov/observe-the-moon-night/participate/newsletter/) -> Quaterly, NASA updates on lunar exploration [Free Signup]
* [Moon Monday](https://blog.jatan.space/s/moon-monday/archive) -> Weekly updates on global lunar exploration activities (comprehensive) [Free Signup]
* [Orbital Index](https://orbitalindex.com/) -> Weekly newsletter curating space news and links, often includes GNC-relevant topics and open source projects. [Free Signup]
* [Payload Space](https://payloadspace.com/) -> Daily newsletter covering the business and policy of space, often relevant context for lunar missions. [Free Signup]

## Online communities

* [Reddit r/space](https://www.reddit.com/r/space/) -> General space news and discussion. [Free]
* [Reddit r/aerospaceengineering](https://www.reddit.com/r/AerospaceEngineering/) -> More technical discussions, including GNC topics. [Free]
* [Reddit r/robotics](https://www.reddit.com/r/robotics/) -> Relevant for rover GNC, simulation, perception, and control. [Free]
* [Stack Exchange - Space Exploration](https://space.stackexchange.com/) -> Q&A site for spaceflight questions, including GNC. [Free]
* [Stack Exchange - Robotics](https://robotics.stackexchange.com/) -> Q&A site for robotics, relevant for rover GNC. [Free]
* [NASA Space Network (NSN) Forum](https://nsn.jpl.nasa.gov/interactions/forum/) -> Official forum, sometimes has relevant discussions. [Free Signup Likely Required]
* [Specific project communities (e.g., Orekit Forum, ROS Discourse)](https://forum.orekit.org/, https://discourse.ros.org/) -> Communities built around specific tools listed elsewhere in this document. [Free]

## Books & Papers

*Note: [OA] indicates Open Access or freely available resource.*

**Textbooks & Foundational Works (Often Paid, Check Libraries):**

*   [(Aerospace Series) Ashish Tewari-Advanced Control of Aircraft, Spacecraft and Rockets-Wiley (2011)](https://www.wiley.com/en-us/Advanced+Control+of+Aircraft%2C+Spacecraft+and+Rockets-p-9780470745632) -> Optimal attitude control, Guidance, Terminal Guidance for spacecraft with MATLAB® codes.
*   [Sidi, M. (1997). Spacecraft Dynamics and Control: A Practical Engineering Approach (Cambridge Aerospace Series). Cambridge: Cambridge University Press. doi:10.1017/CBO9780511815652](https://www.cambridge.org/core/books/spacecraft-dynamics-and-control/E9CAEE81CD09527C99497FA8C7C35B0A) -> Great book for attitude control using thrusters.
*   [Wie, Bong. (2008). Space Vehicle Dynamics and Control (2nd ed.). AIAA Education Series.](https://arc.aiaa.org/doi/book/10.2514/4.866875) -> Comprehensive textbook covering attitude dynamics, orbital dynamics, and control.
*   [Hughes, Peter C. (1986). Spacecraft Attitude Dynamics. Dover Publications.](https://store.doverpublications.com/0486481492.html) -> Classic text focusing specifically on attitude dynamics and control. (Dover editions are often affordable).
*   [Battin, Richard H. (1999). An Introduction to the Mathematics and Methods of Astrodynamics. AIAA Education Series.](https://arc.aiaa.org/doi/book/10.2514/4.868823) -> Foundational text on astrodynamics, essential for GNC.
*   [Vallado, David A. (2013). Fundamentals of Astrodynamics and Applications (4th ed.). Microcosm Press.](https://www.goodreads.com/book/show/1727996.Fundamentals_of_Astrodynamics_and_Applications) -> Widely used textbook and reference, often comes with software. Check website for potential code snippets.

**Open Access Papers, Reports & Resources:**

*   [NASA NTRS (NASA Technical Reports Server)](https://ntrs.nasa.gov/) -> [OA] Searchable database of NASA technical reports. Crucial for historical and modern GNC work. Examples:
    *   Search: "Apollo Guidance Computer", "Lunar Module Descent Guidance", "Terrain Relative Navigation NASA", "Precision Lunar Landing NTRS".
    *   [Apollo Experience Reports (Series)](https://ntrs.nasa.gov/search?q=apollo%20experience%20report) -> [OA] Contain detailed operational insights, including GNC aspects.
    *   [Guidance and Control for Lunar Module Touchdown (1967)](https://ntrs.nasa.gov/citations/19670026403) -> [OA] Example historical paper.
    *   [Autonomous Precision Landing and Hazard Avoidance for Lunar Missions (Multiple Papers)](https://ntrs.nasa.gov/search?q=autonomous%20precision%20landing%20lunar) -> [OA] Search results for more recent NASA work.
*   [JPL Robotics - Publications](https://www-robotics.jpl.nasa.gov/publications/index.cfm) -> [OA] Publications from JPL on robotics, relevant for rover GNC. Often links to PDF versions.
*   [arXiv - Physics > Space Physics / Instrumentation & Methods for Astrophysics](https://arxiv.org/list/physics.space-ph/recent) -> [OA] Pre-print server where researchers post papers before/alongside journal publication. Search for relevant keywords.
*   [ResearchGate / Academia.edu](https://www.researchgate.net/, https://www.academia.edu/) -> Researchers often upload their papers. Access may require signup. Search for specific authors or topics.

**Mission Specific Strategies (Open Access Info):**

*   **Apollo:**
    *   [Mindell, David A. (2008). Digital Apollo: Human and Machine in Spaceflight. MIT Press.](https://mitpress.mit.edu/9780262633930/digital-apollo/) (Book, check libraries). Excellent analysis of Apollo GNC development and human-machine interaction.
    *   [Eyles, Don. Sunburst and Luminary: An Apollo Memoir.](https://www.doneyles.com/LM/Tales.html) [OA via Author's Site] Personal accounts and technical details from an AGC programmer.
    *   [Apollo Lunar Descent and Ascent Trajectories (NASA TN D-6846)](https://ntrs.nasa.gov/citations/19720017001) -> [OA via NTRS] Detailed trajectory analysis.
*   **Soviet Luna Program:**
    *   [Harvey, Brian. Soviet and Russian Lunar Exploration (2007). Springer-Praxis.](https://link.springer.com/book/10.1007/978-0-387-73976-2) (Book, check libraries). Comprehensive overview including landing techniques where known.
    *   [Siddiqi, Asif A. Challenge to Apollo: The Soviet Union and the Space Race, 1945-1974 (NASA SP-2000-4408)](https://history.nasa.gov/SP-4408pt1.pdf) -> [OA via NASA History] Part 1 & 2. Contains details on Luna lander development and mission attempts, including GNC aspects derived from historical analysis.
    *   Analysis of Luna 9/13/etc. landings can often be found in historical reviews and comparative papers on NTRS or conference archives (search terms like "Luna 9 landing analysis").
*   **Chinese Chang'e Program:**
    *   [Li, Chunlai, et al. (2021). China’s Lunar Exploration Program: Present and Future. Space Science Reviews.](https://link.springer.com/article/10.1007/s11214-021-00841-4) -> [OA via SpringerLink] Overview paper, mentions GNC advancements like hazard avoidance.
    *   [Guidance Navigation and Control for Chang’E-5 Powered Descent and Ascent Mission (Journal of Spacecraft & Rockets, 2021)](https://arc.aiaa.org/doi/abs/10.2514/1.A35018) -> (AIAA Paper, likely requires payment/subscription). Abstract summarizes GNC strategies. Look for related conference presentations by authors (e.g., IAC) which might be open.
    *   [Landing Site Selection and Traverse Planning for the Chang’e-4 Lander and Rover (Nature Astronomy, 2020)](https://www.nature.com/articles/s41550-019-0813-z) -> (Nature Paper, may require payment/subscription). Describes GNC-related aspects like hazard detection and safe site selection logic.
    *   [Autonomous hazard avoidance control for Chang'e-3 lunar lander (Science China Technological Sciences, 2014)](https://link.springer.com/article/10.1007/s11431-014-5501-9) -> [OA via SpringerLink] Describes the hazard detection and avoidance strategy used for CE-3.
*   **Indian Chandrayaan Program:**
    *   [ISRO Chandrayaan-3 Landing - Official Livestream and Commentary](https://www.youtube.com/watch?v=DLA_64yz8Ss) -> [OA via YouTube] While not a paper, the official commentary during landing describes the phases and GNC actions (rough braking, altitude hold, fine braking, vertical descent).
    *   [Geethaprasad, G. L., & Nagesh, R. (2023). Chandrayaan-3: India's successful lunar landing mission. Current Science.](https://www.currentscience.ac.in/Volumes/125/05/0525.pdf) -> [OA via Current Science] Overview paper describing mission phases and GNC elements post-landing.
    *   [Analysis of Chandrayaan-2 Vikram Lander Trajectory (Multiple independent analyses exist, e.g., on arXiv or blogs)](https://arxiv.org/abs/1909.10608) -> [OA via arXiv example] Example of third-party analysis trying to reconstruct GNC events based on telemetry. Search for similar papers.
    *   [ISRO Technical Reports/Presentations:] Search the official ISRO website publications section or conference proceedings (like IAC) for presentations by ISRO engineers which might contain more details on specific GNC subsystems or algorithms used in Chandrayaan-3.

## Jobs

* [Careers In Space](https://www.careersin.space/job-listings/?_sft_job_listing_category=guidance-navigation-and-control-gnss) -> Plenty of lunar GNC jobs available, including from upstart space companies
* [Indeed](https://www.indeed.com/jobs?q=Guidance+Navigation+Control+Lunar&l=&vjk=60cb2f00ade4873b) -> Comprehensive for US (and other regions).
* [LinkedIn](https://www.linkedin.com/jobs/search/?keywords=Guidance%20Navigation%20Control%20Lunar) -> Search for GNC jobs globally.
* [Aerospace Corporation Careers](https://aerospace.org/careers) -> Often hires GNC engineers for government oversight roles.
* [NASA Careers](https://www.nasa.gov/careers/) -> Official job portal for NASA.
* [ESA Careers](https://jobs.esa.int/) -> Official job portal for ESA.
* [Specific Company Career Pages] (e.g., SpaceX, Blue Origin, Astrobotic, Intuitive Machines, Firefly Aerospace, Lockheed Martin, Northrop Grumman, Draper) -> Check the career pages of companies actively involved in lunar missions.

## Movers and shakers on Github

* [Time Craine](https://twitter.com/craintim) -> Real-time GNC systems for Intuitive Machines, private moon lander developers (Note: Primarily Twitter/X presence, less direct open-source contribution visible).
* [Chris Rabotin](https://mobile.twitter.com/CRabotin) ([GitLab](https://gitlab.com/chrisrabotin)) -> GNC engineer on various moon landers, developer of Nyx framework.
* [Andrew Annex](https://github.com/AndrewAnnex) -> Developer of SpiceyPy and contributor to related Python space science libraries.
* [Juan Luis Cano Rodríguez](https://github.com/astrojuanlu) -> Core developer of Poliastro and advocate for open-source space software.
* [Jakob Engel](https://github.com/JakobEngel) -> Researcher behind DSO (Direct Sparse Odometry).
* [AVSLab (University of Colorado Boulder)](https://github.com/AVSLab) -> Developers of the Basilisk simulation framework.
* [Moriba Jah](https://github.com/moribajah) ([Website](https://moribajah.com/)) -> Professor at UT Austin, expert in astrodynamics, space traffic management, and orbital safety. His work is crucial for sustainable cislunar operations and orbital GNC.
* [Marco Pavone](https://github.com/StanfordASL) ([Stanford Profile](https://profiles.stanford.edu/marco-pavone)) -> Director of the Autonomous Systems Lab at Stanford. Expert in autonomous systems, robotics, planning, and control relevant to rovers, landers, and orbital servicing. Lab often publishes code on GitHub.
* [Philip Metzger](https://twitter.com/DrPhiltill) ([Google Scholar](https://scholar.google.com/citations?user=Raa56cMAAAAJ&hl=en)) -> Planetary Scientist at UCF, expert on planetary surface processes, regolith mechanics, ISRU, and rocket plume effects. Highly active commentator on lunar development policy and operational challenges.
* [Kevin Cannon](https://github.com/kekannon) ([Website](https://www.kevincannon.science/)) -> Assistant Professor at Colorado School of Mines, focuses on planetary resources (ISRU), surface mapping, and modeling, essential inputs for surface operations and mission planning. Publishes tools and datasets.
* [Ryan Watkins](https://twitter.com/ryanbwatkins) ([PSI Profile](https://www.psi.edu/about/staffpage/rwatkins)) -> Planetary Scientist at PSI, involved in lunar missions (e.g., LRO Diviner). Provides critical scientific context for landing site selection, surface operations, and interpreting remote sensing data used in GNC/Ops. Active commentator.
* [Scott Manley](https://github.com/ScottManley) ([YouTube Channel](https://www.youtube.com/user/szyzyg)) -> Science communicator known for explaining complex spaceflight topics, including orbital mechanics, GNC concepts, and mission analysis, often using tools like KSP and GMAT. Bridges technical concepts with public understanding.
* [Casey Dreier](https://twitter.com/CaseyDreier) ([Planetary Society Profile](https://www.planetary.org/profiles/casey-dreier)) -> Chief Policy Advisor at The Planetary Society. Provides insightful analysis on space policy, NASA budgets, and the strategic context for lunar exploration programs like Artemis.
* [Laura Forczyk](https://twitter.com/LauraForczyk) ([Astralytical Website](https://astralytical.com/)) -> Owner of space consulting firm Astralytical. Analyzes the space industry, including commercial lunar market trends, policy, and mission strategies. Frequent commentator.
* [Chris Lewicki](https://twitter.com/interplanetary) ([LinkedIn](https://www.linkedin.com/in/chrislewicki/)) -> Space investor/advisor, former President & CEO of Planetary Resources. Offers experienced perspectives on commercial space operations, engineering realities, and resource utilization relevant to lunar ventures.
* [Namrata Goswami](https://twitter.com/namratagoswami) ([Website](http://namratagoswami.com/)) -> Independent Scholar & Author focusing on space policy, strategy, and international relations, particularly regarding great power competition in space and lunar ambitions.


## Companies & organisations on Github

* [NASA](https://github.com/nasa) -> Official NASA GitHub account, hosts cFS, GMAT (via SourceForge link), StereoPipeline, and many other projects.
* [ESA](https://github.com/esa) -> Official ESA GitHub account, hosts PLIA and other tools.
* [JPL (Jet Propulsion Laboratory)](https://github.com/nasa-jpl) -> JPL's GitHub presence, often related to robotics, AI, and data processing.
* [Open Lunar Foundation](https://github.com/openlunar) -> Organization promoting open lunar development, hosts lincov tool.
* [Poliastro](https://github.com/poliastro) -> Organization account for the Poliastro library.
* [AVSLab](https://github.com/AVSLab) -> Organization account for the Basilisk framework.
* [ROS (Robot Operating System)](https://github.com/ros) -> Core ROS codebase and related packages.
* [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl) -> Organization account for PCL.
* [OpenCV](https://github.com/opencv) -> Organization account for OpenCV.


<a href="https://www.buymeacoffee.com/chandrabhraman" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/default-orange.png" alt="Buy Me A Coffee" height="41" width="174"></a>
