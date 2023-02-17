// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testingdashboard.TestingDashboard;

public class Arm extends SubsystemBase {

  private static Arm m_arm;

  private CANSparkMax m_shoulder;
  private CANSparkMax m_elbow;
  private CANSparkMax m_turret;

  private RelativeEncoder m_shoulderEncoder;
  private RelativeEncoder m_elbowEncoder;
  private RelativeEncoder m_turretEncoder;

  private AnalogInput m_shoulderPot;
  private AnalogInput m_elbowPot;
  private AnalogInput m_turretPot;

  /** Creates a new Arm. */
  private Arm() {
    m_shoulder = new CANSparkMax(RobotMap.A_SHOULDER_MOTOR, MotorType.kBrushless);
    m_elbow = new CANSparkMax(RobotMap.A_ELBOW_MOTOR, MotorType.kBrushless);
    m_turret = new CANSparkMax(RobotMap.A_TURRET_MOTOR, MotorType.kBrushless);

    m_shoulderEncoder = m_shoulder.getEncoder();
    m_elbowEncoder = m_elbow.getEncoder();
    m_turretEncoder = m_turret.getEncoder();

    m_shoulderPot = new AnalogInput(RobotMap.A_SHOULDER_POTENTIOMETER);
    m_elbowPot = new AnalogInput(RobotMap.A_ELBOW_POTENTIOMETER);
    m_turretPot = new AnalogInput(RobotMap.A_TURRET_POTENTIOMETER);

  }

  public static Arm getInstance() {
    if (m_arm == null) {
      m_arm = new Arm();
      TestingDashboard.getInstance().registerSubsystem(m_arm, "Arm");
      TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "ElbowPotVoltage", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "ShoulderPotVoltage", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "TurretPotVoltage", 0);

      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "ElbowEncoderPulses", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "ShoulderEncoderPulses", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "TurretEncoderPulses", 0);
    }
    return m_arm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowPotVoltage", m_elbowPot.getVoltage());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderPotVoltage", m_shoulderPot.getVoltage());
    TestingDashboard.getInstance().updateNumber(m_arm, "TurretPotVoltage", m_turretPot.getVoltage());

    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowEncoderPulses", m_elbowEncoder.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderEncoderPulses", m_shoulderEncoder.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "TurretEncoderPulses", m_turretEncoder.getPosition());
  }
}
