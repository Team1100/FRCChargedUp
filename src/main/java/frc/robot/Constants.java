// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // General Constants
  public static final double DEGREES_PER_REVOLUTION = 360;
  public static final double DEGREES_PER_JOY_SPAN = 180 / 1;
  public static final double SHOULDER_DEGREES_PER_JOY_SPAN = 90 / 1;

  // Joysticks enabled
  public static final boolean ATTACK_THREE_ENABLE = false;
  public static final boolean BUTTON_BOX_ENABLE = true;
  public static final boolean KEYBOARD_BOX_ENABLE = false;
  public static final boolean XBOX_CONTROLLER_DRIVER_ENABLE = true;
  public static final boolean XBOX_CONTROLLER_OPERATOR_ENABLE = true;

  // Xbox Deadband Limit
  public static final double XBOX_DEADBAND_LIMIT = 0.1;

  // Defines Drive constants
  public static final double D_FWD_RATE_LIMIT = 2.8;
  public static final double D_ROT_RATE_LIMIT = 3.2;

  public static final boolean D_ENABLE_RAMP_RATE = false;

  // Defines Hand command constants
  public static final double DEFAULT_INTAKE_CUBE_POWER = 0.5;
  public static final double DEFAULT_INTAKE_CONE_POWER = 0.5;
  public static final double DEFAULT_EXPEL_CUBE_POWER = -0.3;
  public static final double DEFAULT_EXPEL_CONE_POWER = -0.3;

  public static final double HAND_MAX_POWER = 0.5;


  // Defines Arm command constants

  public static final boolean A_ENABLE_SOFTWARE_PID = true;

  public static final double A_TURRET_SOFTWARE_TOLERANCE = 5;
  public static final double A_TURRET_SOFTWARE_P = 0.01;
  public static final double A_TURRET_SOFTWARE_I = 0;
  public static final double A_TURRET_SOFTWARE_D = 0;

  public static final double A_SHOULDER_SOFTWARE_TOLERANCE = 5;
  public static final double A_SHOULDER_SOFTWARE_P = 0.01;
  public static final double A_SHOULDER_SOFTWARE_I = 0;
  public static final double A_SHOULDER_SOFTWARE_D = 0;

  public static final double A_ELBOW_SOFTWARE_TOLERANCE = 5;
  public static final double A_ELBOW_SOFTWARE_P = 0.01;
  public static final double A_ELBOW_SOFTWARE_I = 0;
  public static final double A_ELBOW_SOFTWARE_D = 0;

  public static final double A_WRIST_SOFTWARE_TOLERANCE = 5;
  public static final double A_WRIST_SOFTWARE_P = 0.01;
  public static final double A_WRIST_SOFTWARE_I = 0;
  public static final double A_WRIST_SOFTWARE_D = 0;

  // Note that MAX POWER should be a value between 0 and 1
  public static final double A_TURRET_MAX_POWER = 0.3;
  public static final double A_SHOULDER_MAX_POWER = 0.4;
  public static final double A_ELBOW_MAX_POWER = 0.4;
  public static final double A_WRIST_MAX_POWER = 0.4;

  // Joint angle increments
  public static final double A_TURRET_ANGLE_INCREMENT = 1; // degrees
  public static final double A_SHOULDER_ANGLE_INCREMENT = 0.2; // degrees
  public static final double A_ELBOW_ANGLE_INCREMENT = 0.2; // degrees
  public static final double A_WRIST_ANGLE_INCREMENT = 1; // degrees

  public static final double TURRET_MOTOR_ROTATIONS_PER_REVOLUTION = 250/12;
  public static final double TURRET_DEGREES_PER_PULSE = DEGREES_PER_REVOLUTION / TURRET_MOTOR_ROTATIONS_PER_REVOLUTION;
  public static final double SHOULDER_MOTOR_ROTATIONS_PER_REVOLUTION = 256;
  public static final double SHOULDER_DEGREES_PER_PULSE = DEGREES_PER_REVOLUTION / SHOULDER_MOTOR_ROTATIONS_PER_REVOLUTION;
  public static final double ELBOW_MOTOR_ROTATIONS_PER_REVOLUTION = 256;
  public static final double ELBOW_DEGREES_PER_PULSE = DEGREES_PER_REVOLUTION / ELBOW_MOTOR_ROTATIONS_PER_REVOLUTION; 
  public static final double WRIST_MOTOR_ROTATIONS_PER_REVOLUTION = 100;
  public static final double WRIST_DEGREES_PER_PULSE = DEGREES_PER_REVOLUTION / WRIST_MOTOR_ROTATIONS_PER_REVOLUTION; 

  public static final double TURRET_POT_START_VOLTAGE = 0;
  public static final double TURRET_POT_END_VOLTAGE = 5;
  public static final double TURRET_POT_FULL_SWING_VOLTAGE = TURRET_POT_END_VOLTAGE > TURRET_POT_START_VOLTAGE? TURRET_POT_END_VOLTAGE - TURRET_POT_START_VOLTAGE: TURRET_POT_START_VOLTAGE - TURRET_POT_END_VOLTAGE;
  public static final double TURRET_POT_START_ANGLE = 0;
  public static final double TURRET_POT_END_ANGLE = 360;
  public static final double TURRET_POT_FULL_SWING_ANGLE = TURRET_POT_END_ANGLE > TURRET_POT_START_ANGLE? TURRET_POT_END_ANGLE - TURRET_POT_START_ANGLE: TURRET_POT_START_ANGLE - TURRET_POT_END_ANGLE;
  public static final double TURRET_POT_DEGREES_PER_VOLT = TURRET_POT_FULL_SWING_ANGLE / TURRET_POT_FULL_SWING_VOLTAGE;

  public static final double SHOULDER_POT_LEFT_START_VOLTAGE = 0;
  public static final double SHOULDER_POT_LEFT_END_VOLTAGE = 5;
  public static final double SHOULDER_POT_LEFT_FULL_SWING_VOLTAGE = SHOULDER_POT_LEFT_END_VOLTAGE > SHOULDER_POT_LEFT_START_VOLTAGE? SHOULDER_POT_LEFT_END_VOLTAGE - SHOULDER_POT_LEFT_START_VOLTAGE: SHOULDER_POT_LEFT_START_VOLTAGE - SHOULDER_POT_LEFT_END_VOLTAGE;
  public static final double SHOULDER_POT_LEFT_START_ANGLE = 0;
  public static final double SHOULDER_POT_LEFT_END_ANGLE = 360;
  public static final double SHOULDER_POT_LEFT_FULL_SWING_ANGLE = SHOULDER_POT_LEFT_END_ANGLE > SHOULDER_POT_LEFT_START_ANGLE? SHOULDER_POT_LEFT_END_ANGLE - SHOULDER_POT_LEFT_START_ANGLE: SHOULDER_POT_LEFT_START_ANGLE - SHOULDER_POT_LEFT_END_ANGLE;
  public static final double SHOULDER_POT_LEFT_DEGREES_PER_VOLT = SHOULDER_POT_LEFT_FULL_SWING_ANGLE / SHOULDER_POT_LEFT_FULL_SWING_VOLTAGE;

  public static final double SHOULDER_POT_RIGHT_START_VOLTAGE = 0;
  public static final double SHOULDER_POT_RIGHT_END_VOLTAGE = 5;
  public static final double SHOULDER_POT_RIGHT_FULL_SWING_VOLTAGE = SHOULDER_POT_RIGHT_END_VOLTAGE > SHOULDER_POT_RIGHT_START_VOLTAGE? SHOULDER_POT_RIGHT_END_VOLTAGE - SHOULDER_POT_RIGHT_START_VOLTAGE: SHOULDER_POT_RIGHT_START_VOLTAGE - SHOULDER_POT_RIGHT_END_VOLTAGE;
  public static final double SHOULDER_POT_RIGHT_START_ANGLE = 0;
  public static final double SHOULDER_POT_RIGHT_END_ANGLE = 360;
  public static final double SHOULDER_POT_RIGHT_FULL_SWING_ANGLE = SHOULDER_POT_RIGHT_END_ANGLE > SHOULDER_POT_RIGHT_START_ANGLE? SHOULDER_POT_RIGHT_END_ANGLE - SHOULDER_POT_RIGHT_START_ANGLE: SHOULDER_POT_RIGHT_START_ANGLE - SHOULDER_POT_RIGHT_END_ANGLE;
  public static final double SHOULDER_POT_RIGHT_DEGREES_PER_VOLT = SHOULDER_POT_RIGHT_FULL_SWING_ANGLE / SHOULDER_POT_RIGHT_FULL_SWING_VOLTAGE;

  public static final double ELBOW_POT_LEFT_START_VOLTAGE = 0;
  public static final double ELBOW_POT_LEFT_END_VOLTAGE = 5;
  public static final double ELBOW_POT_LEFT_FULL_SWING_VOLTAGE = ELBOW_POT_LEFT_END_VOLTAGE > ELBOW_POT_LEFT_START_VOLTAGE? ELBOW_POT_LEFT_END_VOLTAGE - ELBOW_POT_LEFT_START_VOLTAGE: ELBOW_POT_LEFT_START_VOLTAGE - ELBOW_POT_LEFT_END_VOLTAGE;
  public static final double ELBOW_POT_LEFT_START_ANGLE = 0;
  public static final double ELBOW_POT_LEFT_END_ANGLE = 360;
  public static final double ELBOW_POT_LEFT_FULL_SWING_ANGLE = ELBOW_POT_LEFT_END_ANGLE > ELBOW_POT_LEFT_START_ANGLE? ELBOW_POT_LEFT_END_ANGLE - ELBOW_POT_LEFT_START_ANGLE: ELBOW_POT_LEFT_START_ANGLE - ELBOW_POT_LEFT_END_ANGLE;
  public static final double ELBOW_POT_LEFT_DEGREES_PER_VOLT = ELBOW_POT_LEFT_FULL_SWING_ANGLE / ELBOW_POT_LEFT_FULL_SWING_VOLTAGE;

  public static final double ELBOW_POT_RIGHT_START_VOLTAGE = 0;
  public static final double ELBOW_POT_RIGHT_END_VOLTAGE = 5;
  public static final double ELBOW_POT_RIGHT_FULL_SWING_VOLTAGE = ELBOW_POT_RIGHT_END_VOLTAGE > ELBOW_POT_RIGHT_START_VOLTAGE? ELBOW_POT_RIGHT_END_VOLTAGE - ELBOW_POT_RIGHT_START_VOLTAGE: ELBOW_POT_RIGHT_START_VOLTAGE - ELBOW_POT_RIGHT_END_VOLTAGE;
  public static final double ELBOW_POT_RIGHT_START_ANGLE = 0;
  public static final double ELBOW_POT_RIGHT_END_ANGLE = 360;
  public static final double ELBOW_POT_RIGHT_FULL_SWING_ANGLE = ELBOW_POT_RIGHT_END_ANGLE > ELBOW_POT_RIGHT_START_ANGLE? ELBOW_POT_RIGHT_END_ANGLE - ELBOW_POT_RIGHT_START_ANGLE: ELBOW_POT_RIGHT_START_ANGLE - ELBOW_POT_RIGHT_END_ANGLE;
  public static final double ELBOW_POT_RIGHT_DEGREES_PER_VOLT = ELBOW_POT_RIGHT_FULL_SWING_ANGLE / ELBOW_POT_RIGHT_FULL_SWING_VOLTAGE;

  // Velocity Conversion Values for the arm motors
  // TODO: Fill in with appropriate conversion factors
  public static final double TURRET_MOTOR_VEL_CONVERSION_FACTOR = 1 / TURRET_MOTOR_ROTATIONS_PER_REVOLUTION;
  public static final double SHOULDER_MOTOR_VEL_CONVERSION_FACTOR = 1 / SHOULDER_MOTOR_ROTATIONS_PER_REVOLUTION;
  public static final double ELBOW_MOTOR_VEL_CONVERSION_FACTOR = 1 / ELBOW_MOTOR_ROTATIONS_PER_REVOLUTION;


  // Defines if systems are availible
  public static final boolean HW_ENABLE_DRIVE = true;
  public static final boolean HW_ENABLE_ARM = true;
  public static final boolean HW_ENABLE_HAND = true;

  // Subsystem periodic loops
  public static final boolean DRIVE_PERIODIC_ENABLE = true;
  public static final double DRIVE_RAMP_RATE = 1; // Limit changes in power to require 2 secons to go from zero to full

  // Motor current limits
  public static final double TURRET_MOTOR_CURRENT_LIMIT = 60;
  public static final double SHOULDER_MOTOR_CURRENT_LIMIT = 60;
  public static final double ELBOW_MOTOR_CURRENT_LIMIT = 60;
  public static final double WRIST_MOTOR_CURRENT_LIMIT = 60;


}
