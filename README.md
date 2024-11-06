# Cluedo-Inspired Robot Exploration in ROS and Gazebo

## Experimental Robotics Laboratory - Assignment II
- Author: Muhammad Ali Haider Dar
- Email: [5046263@studenti.unige.it](mailto:5046263@studenti.unige.it)
- Program: MSc Robotics Engineering, University of Genoa, Italy
- Instructor: [Prof. Carmine Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r)


### Project Overview

This project extends the work done in Assignment 1 of the Experimental Robotics course, which can be found [here](https://github.com/sudohaider/experimental-robotics-a1). In the previous project, a ROS package was developed for a simulation of the Cluedo game, where a robot explored the environment to collect hints and deduced a hypothesis about who might be the killer.

Building upon this architecture, the project features upgrades in both environment simulation and task-motion planning. We have developed a scene in the Gazebo simulator, which includes a custom-made robot model with an arm attached to its base. The environment contains four hovering points with coordinates (-3,0), (3,0), (0,-3), (0,3) for the 'x' and 'y' axes. The 'z' coordinate can be either 0.75 or 1.25, chosen randomly each time. These points represent the locations of four rooms where the robot needs to position its arm’s end-effector to collect hints.

### Environment Details

The simulation also includes small walls which prevent the robot from reaching the points with its mobile base, so the robot must plan its arm motion to place it over the point coordinates to collect hints.

### Hint Types and Hypothesis

Similar to the previous project, the deduced hypotheses must be consistent and correct. This means that the hypotheses should be based on three types of hints, and their IDs must match the ID of the correct hypothesis. The hint types are:

Who: A name of a person who might be the killer (e.g., Prof. Plum).
What: A name of a weapon that the killer might have used (e.g., Dagger).
Where: A name of a place where the crime might have occurred (e.g., Hall).
A consistent hypothesis might state: “Prof. Plum with the Dagger in the Hall.” If the hypothesis is incorrect, the robot will revisit the rooms for new hints until it forms a consistent hypothesis.

### ARMOR Integration

As in the previous project, we use the ARMOR package to deduce the hypothesis. Developed by researchers at the University of Genova, ARMOR is a versatile management system that can handle single or multiple-ontology architectures within ROS. For more details, see the [ARMOR repository](https://github.com/EmaroLab/armor).

### ROSPlan Integration

Additionally, we use ROSPlan to plan the robot's behavior. ROSPlan is a framework that provides a collection of tools for AI Planning in a ROS system. It includes nodes for planning, problem generation, and plan execution. We translated the project's problem statement into a PDDL problem file, which includes objects like 'robot' and 'waypoint', initial conditions, and goals describing the desired final state of the environment. The domain file contains actions the robot can take to achieve the goal, such as 'goto_waypoint'.

At the start of the simulation, we execute the ROSPlan planning loop services, which include problem generation, planning, parsing, and dispatching. During execution, the robot may fail to complete the goals on the first attempt. The project architecture is designed to detect such failures and trigger replanning. After replanning, the robot executes the new plan and continues this process until all goals are achieved.

## Installation

1. Ensure ROS, ARMOR, and ROSPlan are installed:
   - **ARMOR**: [Installation Guide](https://github.com/EmaroLab/armor)
   - **ROSPlan**: [Installation Guide](https://github.com/KCL-Planning/ROSPlan)
2. Clone the repository inside `erl2` folder in your ROS workspace’s `src` directory.
3. Build the package:
    ```bash
   catkin_make
   source devel/setup.bash
    ```
4. To use the Python modules in the `armor_py_api` package, add the path to your `PYTHONPATH` environment variable:
    ```bash
    export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/
    ```
5. Download the `cluedo_ontology.owl` file from this repository and place it in your `/root/Desktop/` directory.
6. Go to the scripts directory and make `rosplan_start.sh` executable.
    ```bash
    cd src/erl2/scripts
    chmod +x rosplan_start.sh
    ```

## Running the Project Simulation

1. Start the ROS master by opening a command window and executing:
    ```bash
    roscore &
    ```
2. Open a new terminal tab and launch the ROS simulation with:
    ```bash
    roslaunch erl2 assignment.launch
    ```
3. Wait for the system to load all files. Once loaded, open another terminal and execute the `assignment_services` launch file:
    ```bash
    roslaunch erl2 assignment_services.launch
    ```
4. Start the ARMOR service using:
    ```bash
    rosrun armor execute it.emarolab.armor.ARMORMainService
    ```
5. After all service nodes are loaded, start the planning and dispatching process by running the `rosplan_start.sh` bash file in another terminal:
    ```bash
    rosrun erl2 rosplan_start.sh
    ```

## Software Architecture

The project architecture consists of the following main nodes:

1. **simulation.cpp**
   - Implements the 'oracle' and visualizes the positions of four hints. The oracle generates random hints and a trustable ID to produce a consistent and correct hypothesis. Hints are published on the `/oracle_hint` topic.

2. **my_action.cpp**
   - Implements an action client for the `goto_waypoint` action in the PDDL domain file. It requests the robot to visit waypoints, adjust orientation, move its arm, and collect hints. Successful hint collection is confirmed by the service returning 'true'.

3. **hint_collector.cpp**
   - Provides the `/request_hint_collector` service, which collects hints from the `/oracle_hint` topic. It checks the consistency of collected hints, loads them into the ARMOR ontology knowledge base, and verifies if the hypothesis is complete and correct.

4. **replan.cpp**
   - Provides the `/request_replan` service, which signals the `replan_sub` node to start the replanning process by publishing a "replan" message on the `/replan` topic.

5. **replan_sub.cpp**
   - Initializes a subscriber for the `/replan` topic. The callback function executes the `rosplan_start.sh` bash file, which triggers the following services as part of the replanning loop:
     - `/rosplan_problem_interface/problem_generation_server`
     - `/rosplan_planner_interface/planning_server`
     - `/rosplan_parsing_interface/parse_plan`
     - `/rosplan_plan_dispatcher/dispatch_plan`

6. **move_arm.cpp**
   - Plans and executes the robot’s arm motion using the ROS MoveIt library. Provides a service for moving the robot arm’s end-effector to a desired point in the Gazebo environment.

7. **hint_loader.py**
   - Waits for requests from the `hint_collector` node to load hints into the ARMOR reasoner. It starts the reasoner, deduces hypotheses, and checks if they are 'COMPLETE' and 'CORRECT', then returns the appropriate response.

8. **set_orientation.py**
   - Receives desired orientation coordinates from the `/request_set_orientation` service and computes the required angular velocity for the robot to achieve the desired orientation. Publishes this velocity to the `cmd_vel` topic.

## Demo Video

The demo video of the project `exprob-a2-demo.mp4` is provided in the repository.

## Documentation

Code documentation is provided in the `docs` folder using the Doxygen tool.
