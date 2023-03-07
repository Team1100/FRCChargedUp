// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testingdashboard.TestingDashboard;

public class ControllerMode extends SubsystemBase {
  /** Creates a new ControllerMode. */

  private static ControllerMode m_controllerMode;

  private ControllerMode() {

  }

  public static ControllerMode getInstance() {
    if (m_controllerMode == null) {
     m_controllerMode = new ControllerMode();
     TestingDashboard.getInstance().registerSubsystem(m_controllerMode, "ControllerMode");
     TestingDashboard.getInstance().registerString(m_controllerMode, "Modes", "CurrentMode", "Mode1");
    }

    return m_controllerMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
