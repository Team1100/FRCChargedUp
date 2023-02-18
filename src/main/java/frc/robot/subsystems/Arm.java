// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TestingDashboard;

public class Arm extends SubsystemBase {

  private static Arm m_arm;
  private static CANSparkMax m_shoulder;
  private static CANSparkMax m_elbow;
  private static CANSparkMax m_turntable;

  private static RelativeEncoder m_turntableEncoder;
  private static RelativeEncoder m_shoulderEncoder;
  private static RelativeEncoder m_elbowEncoder;

  /** Creates a new Arm. */
  private Arm() {}

  public static Arm getInstance() {
    if (m_arm == null) {
      m_arm = new Arm();
      m_turntable = new CANSparkMax(RobotMap.A_TURNTABLE, MotorType.kBrushless);
      m_shoulder = new CANSparkMax(RobotMap.A_SHOULDER, MotorType.kBrushless);
      m_elbow = new CANSparkMax(RobotMap.A_ELBOW, MotorType.kBrushless);

      m_turntableEncoder = m_turntable.getEncoder();
      m_shoulderEncoder = m_shoulder.getEncoder();
      m_elbowEncoder = m_elbow.getEncoder();
      
      TestingDashboard.getInstance().registerSubsystem(m_arm, "Arm");
    }
    return m_arm;
  }

  public double getTurntableAngle() {
    return m_turntableEncoder.getPosition();
  }

  public double getShoulderAngle() {
    return m_shoulderEncoder.getPosition();
  }

  public double getElbowAngle() {
    return m_elbowEncoder.getPosition();
  }

  public void turntableToAngle(double angle) {
    double angleLeft = angle - getTurntableAngle();
    while(angleLeft >= Constants.ANGLE_DEADBAND) {
      m_turntable.set(0.01*(angle-getTurntableAngle()));
    }
  }

  public void shoulderToAngle(double angle) {

  }

  public void elbowToAngle(double angle) {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
