// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hand;

import frc.robot.Constants;
import frc.robot.subsystems.Hand;
import frc.robot.testingdashboard.TestingDashboard;

public class IntakeCube extends SpinIntake {
  /** Creates a new Intake. */
  public IntakeCube(boolean parameterized) {
    super(Constants.DEFAULT_INTAKE_CUBE_POWER, parameterized);
  }

  //Register with TestingDashboard
  public static void registerWithTestingDashboard() {
    Hand hand = Hand.getInstance();
    IntakeCube cmd = new IntakeCube(false);
    TestingDashboard.getInstance().registerCommand(hand, "Basic", cmd);
  }
}
