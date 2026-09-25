# NOTICE — Third-party components and attribution

This repository contains code from the following upstream open-source projects.
Each component keeps its original license and copyright; AeroSLAM-specific code
is licensed under GPL-3.0 (see `LICENSE`).

| Component in AeroSLAM | Upstream project | Version / ref | License | Copyright |
|---|---|---|---|---|
| `src/uav/hector_quadrotor/` | [tu-darmstadt-ros-pkg/hector_quadrotor](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor) | `noetic-devel` @ `9ba4441` | BSD-3-Clause | © 2012–2016 Institute of Flight Systems and Automatic Control, TU Darmstadt |
| `src/uav/hector_gazebo/` | [tu-darmstadt-ros-pkg/hector_gazebo](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo) | `melodic-devel` @ `de947fb` | BSD-3-Clause | © TU Darmstadt |
| `src/uav/hector_models/` | [tu-darmstadt-ros-pkg/hector_models](https://github.com/tu-darmstadt-ros-pkg/hector_models) | `melodic-devel` @ `ebc07c1` | BSD-3-Clause | © TU Darmstadt |
| `src/uav/hector_localization/` | [tu-darmstadt-ros-pkg/hector_localization](https://github.com/tu-darmstadt-ros-pkg/hector_localization) | `catkin` @ `c2c3df1` | BSD-3-Clause | © TU Darmstadt |
| `src/uav/hector_slam/` | [tu-darmstadt-ros-pkg/hector_slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam) | `noetic-devel` @ `a5e77fd` | BSD-3-Clause | © TU Darmstadt |
| `src/common/geographic_info/` | [ros-geographic-info/geographic_info](https://github.com/ros-geographic-info/geographic_info) | `master` @ `bc73c05` | BSD-3-Clause | © Jack O'Quin, Steve Macenski et al. |
| `src/common/unique_identifier/` | [ros-geographic-info/unique_identifier](https://github.com/ros-geographic-info/unique_identifier) | `master` @ `39b8b62` | BSD-3-Clause | © Jack O'Quin et al. |
| `src/common/teleop_twist_keyboard/` | [ros-teleop/teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard) | `master` @ `8e1e14f` | BSD-3-Clause | © Graylin Trevor Jay, Austin Hendrix et al. |
| `src/ground/` | [SYS-zdk/robot_path_planner_public](https://github.com/SYS-zdk/robot_path_planner_public) | public snapshot | GPL-3.0 | © Zhang Dingkun (张定坤) |
| `src/ground/` (upstream framework) | [ai-winter/ros_motion_planning](https://github.com/ai-winter/ros_motion_planning) | — | GPL-3.0 | © Yang Haodong (杨浩东) et al. |
| `thirdparty/LBFGS-Lite/` | LBFGS-Lite | header-only, vendored | MIT | © its authors |

> Notes
> - OSQP (Apache-2.0) is consumed from the ROS package `ros-noetic-osqp-vendor`,
>   glog (BSD-3-Clause) from Ubuntu `libgoogle-glog-dev`; neither is vendored here.
> - Files imported from BSD-licensed upstream projects retain their original
>   copyright headers; modifying those files does not change their license.
> - GPL-3.0 components (`src/ground/`) and AeroSLAM-specific code are covered
>   by the top-level `LICENSE`.
> - When redistributing AeroSLAM, keep this file and all upstream `LICENSE` /
>   copyright headers intact.
