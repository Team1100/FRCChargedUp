// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testingdashboard.TestingDashboard;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Hand extends SubsystemBase {

  private CANSparkMax m_handMotor;

  private static Hand m_hand;

  /** Creates a new Claw. */
  private Hand() {

    m_handMotor = new CANSparkMax(RobotMap.H_MOTOR, MotorType.kBrushless);

    m_handMotor.restoreFactoryDefaults();

  }

  public static Hand getInstance() {
    if (m_hand == null) {
      m_hand = new Hand();
      TestingDashboard.getInstance().registerSubsystem(m_hand, "Hand");
      TestingDashboard.getInstance().registerNumber(m_hand, "MotorInput", "HandPower", Constants.DEFAULT_INTAKE_CUBE_POWER);
      TestingDashboard.getInstance().registerNumber(m_hand, "MotorOutput", "HandOutputCurrent", 0.0d);
    }
    return m_hand;
  }

  public double getHandOutputCurrent() {
    return m_handMotor.getOutputCurrent();
  }

  public void setHandMotorPower(double value) {
    m_handMotor.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
