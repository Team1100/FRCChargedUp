// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testingdashboard.TestingDashboard;

public class Arm extends SubsystemBase {

  private static Arm m_arm;

  /** Creates a new Arm. */
  private Arm() {}

  public static Arm getInstance() {
    if (m_arm == null) {
      m_arm = new Arm();
      TestingDashboard.getInstance().registerSubsystem(m_arm, "Arm");
    }
    return m_arm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
