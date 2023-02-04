// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testingdashboard.TestingDashboard;

public class Hand extends SubsystemBase {

  private static Hand m_hand;

  /** Creates a new Claw. */
  private Hand() {}

  public static Hand getInstance() {
    if (m_hand == null) {
      m_hand = new Hand();
      TestingDashboard.getInstance().registerSubsystem(m_hand, "Hand");
    }
    return m_hand;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
