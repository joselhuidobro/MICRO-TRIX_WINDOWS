# 2025 trix erp
FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=humble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ── ROS + Python + CLI + rqt (GUI) ─────────────────────────────────────
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip python3-dev \
        ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs \
        ros-humble-ros2cli \
        graphviz python3-graphviz \
        # rqt y rqt_graph + Qt
        ros-humble-rqt ros-humble-rqt-graph \
        python3-pyqt5 python3-pyqt5.qtsvg \
        # utilidades X11 (opcional, útiles para probar GUI)
        x11-apps \
        # drivers OpenGL en software (evita errores Qt en contenedores)
        mesa-utils libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*


# ── Python deps de tu app ──────────────────────────────────────────────
COPY app/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt || \
    python3 -m pip install --no-cache-dir \
      flask gunicorn pymodbus orjson flask-sock hypercorn simple-websocket \
      prometheus-client prometheus-flask-exporter
RUN pip install --no-cache-dir docker==6.1.3 redis==5.0.6
RUN pip install --no-cache-dir docker requests-unixsocket

# ── App ────────────────────────────────────────────────────────────────
COPY app/ /app/app/
ENV PYTHONUNBUFFERED=1 PYTHONPATH=/app/app \
    QT_X11_NO_MITSHM=1
WORKDIR /app/app

# IMPORTANTE: no pongas CMD que haga "source" aquí; usa /ros_entrypoint.sh desde docker-compose.

#Architecture and Design

  #  Guidelines - How to architect ROS-based systems.
  #  yEd - A powerful desktop application that can be used to quickly and effectively generate high-quality diagrams.
  #  yed_py - Generates graphML that can be opened in yEd.
  #  Plantuml - Web application to generate UML diagrams on-the-fly in your live documentation.
  #  rqt_graph - Provides a GUI plugin for visualizing the ROS computation graph.
  #  rqt_launchtree - An RQT plugin for hierarchical launchfile configuration introspection.
  #  cpp-dependencies - Tool to check C++ #include dependencies (dependency graphs created in .dot format).
  #  pydeps - Python Module Dependency graphs.
  #  aztarna - A footprinting tool for robots.
  #  draw.io - A free online diagram software for making flowcharts, process diagrams, org charts, UML, ER and network diagrams.
  #  vscode-drawio - This extension integrates Draw.io into VS Code.
  #  Architecture_Decision_Record - A document that captures an important architectural decision made along with its context and consequences.
#   https://github.com/araffin/robotics-rl-srl


# Documentation and Presentation

  #  Typora - A Minimalist Markdown Editor.
  #  Markor - A Simple Markdown Editor for your Android Device.
  #  Pandoc - Universal markup converter.
  #  Yaspeller - Command line tool for spell checking.
  #  ReadtheDocs - Build your local ReadtheDocs Server.
  #  Doxygen - Doxygen is the de facto standard tool for generating documentation from annotated C++ sources.
  #  Sphinx - A tool that makes it easy to create intelligent and beautiful documentation for Python projects.
  #  Word-to-Markdown - A ruby gem to liberate content from Microsoft Word document.
  #  paperless - Index and archive all of your scanned paper documents.
  #  carbon - Share beautiful images of your source code.
  #  undraw - Free Professional business SVGs easy to customize.
  #  asciinema - Lets you easily record terminal sessions and replay them in a terminal as well as in a web browser.
  #  inkscape - Inkscape is a professional vector graphics editor for Linux, Windows and macOS.
  #  Reveal-Hugo - A Hugo theme for Reveal.js that makes authoring and customization a breeze. With it, you can turn any properly-formatted Hugo content into a HTML presentation.
  #  Hugo-Webslides - This is a Hugo template to create WebSlides presentation using markdown.
  #  jupyter2slides - Cloud Native Presentation Slides with Jupyter Notebook + Reveal.js.
  #  patat - Terminal-based presentations using Pandoc.
  #  github-changelog-generator - Automatically generate change log from your tags, issues, labels and pull requests on GitHub.
  #  GitLab-Release-Note-Generator - A Gitlab release note generator that generates release note on latest tag.
  #  OCRmyPDF - Adds an OCR text layer to scanned PDF files, allowing them to be searched.
  #  papermill - A tool for parameterizing, executing, and analyzing Jupyter Notebooks.
  #  docsy - An example documentation site using the Docsy Hugo theme.
  #  actions-hugo - Deploy website based on Hugo to GitHub Pages.
  #  overleaf - An open-source online real-time collaborative LaTeX editor.
  #  landslide - Generate HTML5 slideshows from markdown, ReST, or textile.
  #  libreoffice-impress-templates - Freely-licensed LibreOffice Impress templates.
  #  opensourcedesign - Community and Resources for Free Design and Logo Creation.
  #  olive - A free non-linear video editor aiming to provide a fully-featured alternative to high-end professional video editing software.
  #  buku - Browser-independent bookmark manager.
  #  swiftlatex - A WYSIWYG Browser-based LaTeX Editor.
  #  ReLaXed - Allows complex PDF layouts to be defined with CSS and JavaScript, while writing the content in a friendly, minimal syntax close to Markdown or LaTeX.
  #  foam - Foam is a personal knowledge management and sharing system inspired by Roam Research, built on Visual Studio Code and GitHub.
  #  CodiMD - Open Source Online Real-time collaborate on team documentation in markdown.
  #  jupyter-book - Build interactive, publication-quality documents from Jupyter Notebooks.
  #  InvoiceNet - Deep neural network to extract intelligent information from invoice documents.
  #  tesseract - Open Source OCR Engine.
  #  mkdocs - A fast, simple and downright gorgeous static site generator that's geared towards building project documentation.
  #  PlotNeuralNet - Latex code for drawing neural networks for reports and presentation.
  #  Excalidraw - Virtual whiteboard for sketching hand-drawn like diagrams.
  #  SVGrepo - Download free SVG Vectors for commercial use.
  #  gollum - A simple, Git-powered wiki with a sweet API and local frontend.
  #  GanttLab - The easy to use, fully functional Gantt chart for GitLab and GitHub.
  #  Zotero - A free, easy-to-use tool to help you collect, organize, cite, and share your research sources.

  # Communication and Coordination

  #  Agile Development - Manifesto for Agile Software Development.
  #  Gitflow - Makes parallel development very easy, by isolating new development from finished work.
  #  DeepL - An online translator that outperforms Google, Microsoft and Facebook.
  #  Taiga - Agile Projectmanagment Tool.
  #  Kanboard - Minimalistic Kanban Board.
  #  kanban - Free, open source, self-hosted, Kanban board for GitLab issues.
  #  Gitlab - Simple Selfhosted Gitlab Server with Docker.
  #  Gogs - Build a simple, stable and extensible self-hosted Git service that can be setup in the most painless way.
  #  Wekan - Meteor based Kanban Board.
  #  JIRA API - Python Library for REST API of Jira.
  #  Taiga API - Python Library for REST API of Taiga.
  #  Chronos-Timetracker - Desktop client for JIRA. Track time, upload worklogs without a hassle.
  #  Grge - Grge is a daemon and command line utility augmenting GitLab.
  #  gitlab-triage - Gitlab's issues and merge requests triage, automated.
  #  Helpy - A modern, open source helpdesk customer support application.
  #  ONLYOFFICE - A free open source collaborative system developed to manage documents, projects, customer relationship and email correspondence, all in one place.
  #  discourse - A platform for community discussion. Free, open, simple.
  #  Gerrit - A code review and project management tool for Git based projects.
  #  jitsi-meet - Secure, Simple and Scalable Video Conferences that you use as a standalone app or embed in your web application.
  #  mattermost - An open source, private cloud, Slack-alternative.
  #  openproject - The leading open source project management software.
  #  leantime - Leantime is a lean project management system for innovators.
  #  gitter - Gitter is a chat and networking platform that helps to manage, grow and connect communities through messaging, content and discovery.

   # Debugging and Tracing

    # heaptrack - Traces all memory allocations and annotates these events with stack traces.
    # ros2_tracing - Tracing tools for ROS 2.
     #Linuxperf - Various Linux performance material.
     # lptrace - It lets you see in real-time what functions a Python program is running.
     # pyre-check - Performant type-checking for python.
    # FlameGraph - Visualize profiled code.
    # gpuvis - GPU Trace Visualizer.
    # sanitizer - AddressSanitizer, ThreadSanitizer, MemorySanitizer.
    # cppinsights - C++ Insights - See your source code with the eyes of a compiler.
    # inspect - The inspect module provides functions for learning about live objects, including modules, classes, instances, functions, and methods.
    # Roslaunch Nodes in Valgrind or GDB - When debugging roscpp nodes that you are launching with roslaunch, you may wish to launch the node in a debugging program like gdb or valgrind instead.
    # pyperformance - Python Performance Benchmark Suite.
    # qira - QIRA is a competitor to strace and gdb.
    # gdb-frontend - GDBFrontend is an easy, flexible and extensionable gui debugger.
    # lttng - An open source software toolkit which you can use to simultaneously trace the Linux kernel, user applications, and user libraries.
    # ros2-performance - Allows to easily create arbitrary ROS2 systems and then measures their performance.
    # bcc - Tools for BPF-based Linux IO analysis, networking, monitoring, and more.
    # tracy - A real time, nanosecond resolution, remote telemetry frame profiler for games and other applications.
    # bpftrace - High-level tracing language for Linux eBPF.
    # pudb - Full-screen console debugger for Python.
    # backward-cpp - A beautiful stack trace pretty printer for C++.
    # gdb-dashboard - GDB dashboard is a standalone .gdbinit file written using the Python API that enables a modular interface showing relevant information about the program being debugged.
    # hotspot - The Linux perf GUI for performance analysis.
     
    # memory_profiler - A python module for monitoring memory consumption of a process as well as line-by-line analysis of memory consumption for python programs.
    # ros1_fuzzer - This fuzzer aims to help developers and researchers to find bugs and vulnerabilities in ROS nodes by performing fuzz tests over topics that the target nodes process.
    # vscode-debug-visualizer - An extension for VS Code that visualizes data during debugging.
    # action-tmate - Debug your GitHub Actions via SSH by using tmate to get access to the runner system itself.
    # libstatistics_collector - ROS 2 library providing classes to collect measurements and calculate statistics across them.
    # system_metrics_collector - Lightweight, real-time system metrics collector for ROS2 systems.

 #Version Control

   #  git-fuzzy - A CLI interface to git that relies heavily on fzf.
    # meld - Meld is a visual diff and merge tool that helps you compare files, directories, and version controlled projects.
    # tig - Text-mode interface for git.
    # gitg - A graphical user interface for git.
    # git-cola - The highly caffeinated Git GUI.
    # python-gitlab - A Python package providing access to the GitLab server API.
    # bfg-repo-cleaner - Removes large or troublesome blobs like git-filter-branch does, but faster.
    # nbdime - Tools for diffing and merging of Jupyter notebooks.
    # semantic-release - Fully automated version management and package publishing.
    # go-semrel-gitab - Automate version management for Gitlab.
    # Git-repo - Git-Repo helps manage many Git repositories, does the uploads to revision control systems, and automates parts of the development workflow.
    # dive - A tool for exploring each layer in a docker image.
    # dvc - Management and versioning of datasets and machine learning models.
    # learnGitBranching - A git repository visualizer, sandbox, and a series of educational tutorials and challenges.
    # gitfs - You can mount a remote repository's branch locally, and any subsequent changes made to the files will be automatically committed to the remote.
    # git-secret - Encrypts files with permitted users' public keys, allowing users you trust to access encrypted data using pgp and their secret keys.
    # git-sweep - A command-line tool that helps you clean up Git branches that have been merged into master.
    # lazygit - A simple terminal UI for git commands, written in Go with the gocui library.
    #glab - An open-source GitLab command line tool.

 #Simulation

   #  AI2-THOR - Python framework with a Unity backend providing interaction, navigation, and manipulation support for household based robotic agents, consisting of 200+ of custom scenes, 1500+ custom annotated objects, and 200+ actions.
    # Drake - Drake aims to simulate even very complex dynamics of robots.
    # Webots - Webots is an open source robot simulator compatible (among others) with ROS and ROS2.
    # lgsv - LG Electronics America R&D Center has developed an HDRP Unity-based multi-robot simulator for autonomous vehicle developers.
    # carla - Open-source simulator for autonomous driving research.
    # awesome-CARLA - A curated list of awesome CARLA tutorials, blogs, and related projects.
    # ros-bridge - ROS bridge for CARLA Simulator.
    # scenario_runner - Traffic scenario definition and execution engine.
    # deepdive - End-to-end simulation for self-driving cars.
    # uuv_simulator - Gazebo/ROS packages for underwater robotics simulation.
    # AirSim - Open source simulator for autonomous vehicles built on Unreal Engine.
    # self-driving-car-sim - A self-driving car simulator built with Unity.
    #ROSIntegration - Unreal Engine Plugin to enable ROS Support.
    # gym-gazebo - An OpenAI gym extension for using Gazebo known as gym-gazebo.
    # gym-pybullet-drones - PyBullet-based Gym environments for single and multi-agent reinforcement learning of quadcopter control.
    # safe-control-gym - PyBullet-based CartPole and Quadrotor environments—with CasADi symbolic dynamics and constraints—for safe and robust learning-based control.
    # highway-env - A collection of environments for autonomous driving and tactical decision-making tasks.
    # VREP Interface - ROS Bridge for the VREP simulator.
    # car_demo - This is a simulation of a Prius in gazebo 9 with sensor data being published using ROS kinetic.
    # sumo - Eclipse SUMO is an open source, highly portable, microscopic and continuous road traffic simulation package designed to handle large road networks.
    # open-simulation-interface - A generic interface for the environmental perception of automated driving functions in virtual scenarios.
    # ESIM - An Open Event Camera Simulator.
    # Menge - Crowd Simulation Framework.
    # pedsim_ros - Pedestrian simulator powered by the social force model for Gazebo.
    # opencrg - Open file formats and open source tools for the detailed description, creation and evaluation of road surfaces.
    # esmini - A basic OpenSCENARIO player.
    # OpenSceneGraph - An open source high performance 3D graphics toolkit, used by application developers in fields such as visual simulation, games, virtual reality, scientific visualization and modelling.
    # morse - An academic robotic simulator, based on the Blender Game Engine and the Bullet Physics engine.
   #  ROSIntegrationVision - Support for ROS-enabled RGBD data acquisition in Unreal Engine Projects.
   #  fetch_gazebo - Contains the Gazebo simulation for Fetch Robotics Fetch and Freight Research Edition Robots.
   #  rotors_simulator - Provides some multirotor models.
   # flow - A computational framework for deep RL and control experiments for traffic microsimulation.
  #  gnss-ins-sim - GNSS + inertial navigation, sensor fusion simulator. Motion trajectory generator, sensor models, and navigation.
  #  Ignition Robotics - Test control strategies in safety, and take advantage of simulation in continuous integration tests.
  #  simulation assets for the SubT - This collection contains simulation assets for the SubT Challenge Virtual Competition in Gazebo.
  #  gazebo_ros_motors - Contains currently two motor plugins for Gazebo, one with an ideal speed controller and one without a controller that models a DC motor.
  #  map2gazebo - ROS package for creating Gazebo environments from 2D maps.
  #  sim_vehicle_dynamics - Vehicle Dynamics Simulation Software of TUM Roborace Team.
  #  gym-carla - An OpenAI gym wrapper for CARLA simulator.
  #  simbody - High-performance C++ multibody dynamics/physics library for simulating articulated biomechanical and mechanical systems like vehicles, robots, and the human skeleton.
  #  gazebo_models - This repository holds the Gazebo model database.
   #  pylot - Autonomous driving platform running on the CARLA simulator.
    # flightmare - Flightmare is composed of two main components: a configurable rendering engine built on Unity and a flexible physics engine for dynamics simulation.
    # champ - ROS Packages for CHAMP Quadruped Controller.
    # rex-gym - OpenAI Gym environments for an open-source quadruped robot (SpotMicro).
    # Trick - Developed at the NASA Johnson Space Center, is a powerful simulation development framework that enables users to build applications for all phases of space vehicle development.
   #  usv_sim_lsa - Unmanned Surface Vehicle simulation on Gazebo with water current and winds.
   #  42 - Simulation for spacecraft attitude control system analysis and design.
   #  Complete_Street_Rule - A scenario oriented design tool intended to enable users to quickly create procedurally generated multimodal streets in ArcGIS CityEngine.
   #  AutoCore simulation - Provides test environment for Autoware and still during early development, contents below may changed during updates.
   #  fields-ignition - Generate random crop fields for Ignition Gazebo.
   #  Unity-Robotics-Hub - Central repository for tools, tutorials, resources, and documentation for robotic simulation in Unity.
   #  BlueSky - The goal of BlueSky is to provide everybody who wants to visualize, analyze or simulate air traffic with a tool to do so without any restrictions, licenses or limitations.
   #  Cloe - Empowers developers of automated-driving software components by providing a unified interface to closed-loop simulation.
   #  Dynamic_logistics_Warehouse - Gazebo simulation of dynamics environment in warehouses.
 #    OpenCDA - A generalized framework for prototyping full-stack cooperative driving automation applications under CARLA+SUMO.

 #Electronics and Mechanics

   #  HRIM - An information model for robot hardware.
   #  URDF - Repository for Unified Robot Description Format (URDF) parsing code.
   #  phobos - An add-on for Blender allowing to create URDF, SDF and SMURF robot models in a WYSIWYG environment.
   #  urdf-viz - Visualize URDF/XACRO file, URDF Viewer works on Windows/macOS/Linux.
   #  solidworks_urdf_exporter - SolidWorks to URDF Exporter.
   #  FreeCAD - Your own 3D parametric modeler.
   #  kicad - A Cross Platform and Open Source Electronics Design Automation Suite.
   #  PcbDraw - Convert your KiCAD board into a nice looking 2D drawing suitable for pinout diagrams.
   #  kicad-3rd-party-tools - Tools made by others to augment the KiCad PCB EDA suite.
   #  PandaPower - An easy to use open source tool for power system modeling, analysis and optimization with a high degree of automation.
   #  LibrePCB - A powerful, innovative and intuitive EDA tool for everyone.
   #  openscad - A software for creating solid 3D CAD models.
  #   ngspice - A open source spice simulator for electric and electronic circuits.
  #   GNSS-SDR - GNSS-SDR provides interfaces for a wide range of radio frequency front-ends and raw sample file formats, generates processing outputs in standard formats.
  #   riscv - The Free and Open RISC Instruction Set Architecture.
  #   urdfpy - A simple and easy-to-use library for loading, manipulating, saving, and visualizing URDF files.
  #   FMPy - Simulate Functional Mockup Units (FMUs) in Python.
  #   FMIKit-Simulink - Import and export Functional Mock-up Units with Simulink.
 #    oemof-solph - A modular open source framework to model energy supply systems.
  #   NASA-3D-Resources - Here you'll find a growing collection of 3D models, textures, and images from inside NASA.
  #   SUAVE - An Aircraft Design Toolbox.
  #   opem - The Open-Source PEMFC Simulation Tool (OPEM) is a modeling tool for evaluating the performance of proton exchange membrane fuel cells.
  #   pvlib-python - A community supported tool that provides a set of functions and classes for simulating the performance of photovoltaic energy systems.
  #   WireViz - A tool for easily documenting cables, wiring harnesses and connector pinouts.
  #   Horizon - EDA is an Electronic Design Automation package supporting an integrated end-to-end workflow for printed circuit board design including parts management and schematic entry.
  #   tigl - The TiGL Geometry Library can be used for the computation and processing of aircraft geometries stored inside CPACS files.
  #   foxBMS - A free, open and flexible development environment to design battery management systems.
  #   cadCAD - A Python package that assists in the processes of designing, testing and validating complex systems through simulation, with support for Monte Carlo methods, A/B testing and parameter sweeping.
  #   OpenMDAO - An open-source framework for efficient multidisciplinary optimization.
  #   ODrive - The aim is to make it possible to use inexpensive brushless motors in high performance robotics projects.
  #   OpenTirePython - An open-source mathematical tire modelling library.
  #   Inkscape Ray Optics - An extension for Inkscape that makes it easier to draw optical diagrams.
  #   OpenAeroStruct - A lightweight tool that performs aerostructural optimization using OpenMDAO.

#  Sensor Processing
# Calibration and Transformation

 #   tf2 - Transform library, which lets the user keep track of multiple coordinate frames over time.
  #  TriP - A Inverse Kinematics library for serial robots, parallel robots and hybrids of both.
  #  lidar_align - A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor.
  #  kalibr - The Kalibr visual-inertial calibration toolbox.
  #  Calibnet - Self-Supervised Extrinsic Calibration using 3D Spatial Transformer Networks.
  #  lidar_camera_calibration - ROS package to find a rigid-body transformation between a LiDAR and a camera.
  #  ILCC - Reflectance Intensity Assisted Automatic and Accurate Extrinsic Calibration of 3D LiDAR.
  #  easy_handeye - Simple, straighforward ROS library for hand-eye calibration.
  #  imu_utils - A ROS package tool to analyze the IMU performance.
  #  kalibr_allan - IMU Allan standard deviation charts for use with Kalibr and inertial kalman filters.
  #  pyquaternion - A full-featured Python module for representing and using quaternions.
  #  robot_calibration - This package offers calibration of a number of parameters of a robot, such as: 3D Camera intrinsics, extrinsics Joint angle offsets and robot frame offsets.
  #  multi_sensor_calibration - Contains a calibration tool to calibrate a sensor setup consisting of lidars, radars and cameras.
  #  LiDARTag - A Real-Time Fiducial Tag using Point Clouds Lidar Data.
  #  multicam_calibration - Extrinsic and intrinsic calbration of cameras.
  #  ikpy - An Inverse Kinematics library aiming performance and modularity.
  #  livox_camera_lidar_calibration - Calibrate the extrinsic parameters between Livox LiDAR and camera.
  #  lidar_camera_calibration - Camera LiDAR Calibration using ROS, OpenCV, and PCL.
  #  e2calib - Contains code that implements video reconstruction from event data for calibration.

#Perception Pipeline

 #   SARosPerceptionKitti - ROS package for the Perception (Sensor Processing, Detection, Tracking and Evaluation) of the KITTI Vision Benchmark Suite.
  #  multiple-object-tracking-lidar - C++ implementation to Detect, track and classify multiple objects using LIDAR scans or point cloud.
  #  cadrl_ros - ROS package for dynamic obstacle avoidance for ground robots trained with deep RL.
  #  AugmentedAutoencoder - RGB-based pipeline for object detection and 6D pose estimation.
  #  jsk_recognition - A stack for the perception packages which are used in JSK lab.
  #  GibsonEnv - Gibson Environments: Real-World Perception for Embodied Agents.
  #  morefusion - Multi-object Reasoning for 6D Pose Estimation from Volumetric Fusion.
  #  se(3)-TrackNet - A package for 6D pose tracking of dynamic objects when object's CAD model is available.

# Machine Learning
# DLIB - A toolkit for making real world machine learning and data analysis applications in C++.
# fastai - The fastai library simplifies training fast and accurate neural nets using modern best practices.
# tpot - A Python Automated Machine Learning tool that optimizes machine learning pipelines using genetic programming.
# deap - Distributed Evolutionary Algorithms in Python.
# gym - A toolkit for developing and comparing reinforcement learning algorithms.
# tensorflow_ros_cpp - A ROS package that allows to do Tensorflow inference in C++ without the need to compile TF yourself.
# Tensorflow Federated - TensorFlow Federated (TFF) is an open-source framework for machine learning and other computations on decentralized data.
# finn - Fast, Scalable Quantized Neural Network Inference on FPGAs.
# neuropod - Neuropod is a library that provides a uniform interface to run deep learning models from multiple frameworks in C++ and Python.
# leela-zero - This is a fairly faithful reimplementation of the system described in the Alpha Go Zero paper "Mastering the Game of Go without Human Knowledge".
# Trax - A library for deep learning that focuses on sequence models and reinforcement learning.
# mlflow - A platform to streamline machine learning development, including tracking experiments #, packaging code into reproducible runs, and sharing and deploying models.
 #Netron - Visualizer for neural network, deep learning and machine learning models.
 #MNN - A blazing fast, lightweight deep learning framework, battle-tested by business-critical use cases in Alibaba.
 #Tensorforce - An open-source deep reinforcement learning framework, with an emphasis on modularized flexible library design and straightforward usability for applications in research and practice.
 #Dopamine - A res