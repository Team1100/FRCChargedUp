// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TestingDashboard;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_intakeWinch;

  private static Intake m_intake;

  public Intake() {

    m_intakeMotor = new CANSparkMax(RobotMap.I_MOTOR_LEFT, MotorType.kBrushless);
    m_intakeWinch = new CANSparkMax(RobotMap.I_MOTOR_RIGHT, MotorType.kBrushless);

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeWinch.restoreFactoryDefaults();

    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_intakeWinch.setIdleMode(IdleMode.kCoast);

  }

  public static Intake getInstance() {
    if (m_intake == null) {
      m_intake = new Intake();
      TestingDashboard.getInstance().registerSubsystem(m_intake, "Intake");
    }
    return m_intake;
  }

  public void setIntakeMotorPower(double power) {
    m_intakeMotor.set(power);
  }

  public void setIntakeWinchPower(double power) {
    m_intakeWinch.set(power);
  }

  public void setWinchBrake() {
    m_intakeWinch.setIdleMode(IdleMode.kBrake);
  }

  public void setWinchCoast() {
    m_intakeWinch.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
