// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testingdashboard.TestingDashboard;

public class Claw extends SubsystemBase {

  private static Claw m_claw;

  /** Creates a new Claw. */
  private Claw() {}

  public static Claw getInstance() {
    if (m_claw == null) {
      m_claw = new Claw();
      TestingDashboard.getInstance().registerSubsystem(m_claw, "Claw");
    }
    return m_claw;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
