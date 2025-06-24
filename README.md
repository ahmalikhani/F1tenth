# BSc Thesis Supplement – Race Algorithm Development for Simulated and Real Robot in ROS 2
# Created by Amirhossein M.Alikhani #

This supplement contains all Python scripts, launch files, logs, and evaluation tools used in the thesis project. The files are structured to reflect the modular and unified design of the adaptive racing controller built for the Roboracer platform under ROS 2 (Humble).

## Folder Structure

- `/analysis_tools/`  
  Includes data logging and analysis tools for post-processing and metric extraction.  
  - `analyzer.py`, `analyzer_plot.py`, `data_logger.py`, `metrics.py`, `multi_metrics_comparator.py`

- `/controller_features/`  
  Contains all feature-level ROS 2 nodes and core controller logic.  
  - `final_cmd_vel_publisher.py`, `gap_steering_publisher.py`, `adaptive_speed_feature.py`, `adaptive_lookahead_feature.py`, `gp_adaptive_*`

- `/launch/`  
  Contains ROS 2 launch files used for simulation and testing scenarios.  
  - Example: `megoldas2.launch.py`, `racing_gap_follow.launch.py`, `string_rviz_overlay.launch.py`

- `/logs/`  
  Organized logs of each test run (FTG variants) including CSVs and result plots.  
  - `FTG_BaseLine/`, `Adaptive_speed/`, `Adaptive_lookahead/`, `Racing_FTG/`  
  Each folder contains:  
    - `processed_metrics_X.csv`, `summary_test_X.csv`  
    - `steering_plot_test_X.jpg`, `speed_plot_test_X.jpg`, `path_colored_test_X.jpg`

- `/megoldas_sim24/`  
  Source files for the reactive controller node and manual teleoperation.  
  - `follow_the_gap.py`, `joystick_teleop.py`

## Usage Notes

- All scripts are written in Python 3.10 and compatible with ROS 2 Humble.
- To run the system in simulation, use the launch files inside `/launch/`.
- Evaluation plots and metrics are generated using files in `/analysis_tools/`.

For full explanations and performance interpretation, please refer to the main thesis document.
