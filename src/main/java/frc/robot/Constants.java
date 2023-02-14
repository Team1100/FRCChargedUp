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

  // Defines if systems are availible
  public static final boolean HW_ENABLE_DRIVE = true;

  // Subsystem periodic loops
  public static final boolean DRIVE_PERIODIC_ENABLE = true;
  public static final double DRIVE_RAMP_RATE = 2; // Limit changes in power to require 2 secons to go from zero to full

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
