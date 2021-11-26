// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <stdio.h>

#include "v5lvgl.h"
#include "lv_conf.h"

#include "vpi/controller/SimpleTrajectoryFollower.h"
#include "vpi/controller/SimpleWaypointFollower.h"
#include "vpi/controller/RamseteTrajectoryFollower.h"
#include "vpi/trajectory/Trajectory.h"
#include "vpi/trajectory/TrajectoryConfig.h"
#include "vpi/trajectory/TrajectoryGenerator.h"

#pragma once

// define your global instances of motors and other devices here
#define ROBOT_GEAR_SETTING vex::ratio6_1
#define ROBOT_GEAR_RATIO 36.0 / 60.0
#define ROBOT_LEFT_MOTOR_1 PORT1
#define ROBOT_LEFT_MOTOR_1_REV true
#define ROBOT_LEFT_MOTOR_2 PORT19
#define ROBOT_LEFT_MOTOR_2_REV false
#define ROBOT_LEFT_MOTOR_3 PORT10
#define ROBOT_LEFT_MOTOR_3_REV true
#define ROBOT_RIGHT_MOTOR_1 PORT3
#define ROBOT_RIGHT_MOTOR_1_REV true
#define ROBOT_RIGHT_MOTOR_2 PORT20
#define ROBOT_RIGHT_MOTOR_2_REV false
#define ROBOT_RIGHT_MOTOR_3 PORT11
#define ROBOT_RIGHT_MOTOR_3_REV false

#define ROBOT_GPS_PORT PORT8
#define ROBOT_GPS_MOUNT_OFFSET_X 1.25
#define ROBOT_GPS_MOUNT_OFFSET_Y 1.00
#define ROBOT_GPS_MOUNT_OFFSET_HEADING -90
//#define ROBOT_GPS_MOUNT_OFFSET_HEADING 0

#define ROBOT_INERTIAL_PORT PORT5

// You may want to use the PhysicalDriveTuner to help determine these 2 values
#define ROBOT_WHEEL_TRACK 17_in
#define ROBOT_WHEEL_DIAMETER 3.25_in

namespace vpi {
  namespace ui {
    void ramseteInit();
  } // ui
} // vpi
