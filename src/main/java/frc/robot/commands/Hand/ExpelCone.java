// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hand;

import frc.robot.Constants;
import frc.robot.subsystems.Hand;
import frc.robot.testingdashboard.TestingDashboard;

public class ExpelCone extends SpinIntake {
  /** Creates a new ExpelCube. */
  public ExpelCone(boolean parameterized) {
    super(Constants.DEFAULT_EXPEL_CONE_POWER, parameterized);
  }
  
  //Register with TestingDashboard
  public static void registerWithTestingDashboard() {
    Hand hand = Hand.getInstance();
    ExpelCone cmd = new ExpelCone(false);
    TestingDashboard.getInstance().registerCommand(hand, "Basic", cmd);
  }
}
