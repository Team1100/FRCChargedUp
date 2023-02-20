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
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TestingDashboard;

public class Arm extends SubsystemBase {

  private static Arm m_arm;

  private CANSparkMax m_shoulderLeft;
  private CANSparkMax m_shoulderRight;
  private CANSparkMax m_elbowLeft;
  private CANSparkMax m_elbowRight;

  private CANSparkMax m_shoulder;
  private CANSparkMax m_elbow;
  private CANSparkMax m_turret;
  private CANSparkMax m_wrist;



  private RelativeEncoder m_shoulderEncoderLeft;
  private RelativeEncoder m_shoulderEncoderRight;
  private RelativeEncoder m_elbowEncoderLeft;
  private RelativeEncoder m_elbowEncoderRight;
  private RelativeEncoder m_turretEncoder;
  private RelativeEncoder m_wristEncoder;

  private AnalogInput m_shoulderPot;
  private AnalogInput m_elbowPot;
  private AnalogInput m_turretPot;

  /** Creates a new Arm. */
  private Arm() {

    // Initialize ARM motors
    m_shoulderLeft = new CANSparkMax(RobotMap.A_SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
    m_shoulderRight = new CANSparkMax(RobotMap.A_SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
    m_elbowLeft = new CANSparkMax(RobotMap.A_ELBOW_MOTOR_LEFT, MotorType.kBrushless);
    m_elbowRight = new CANSparkMax(RobotMap.A_ELBOW_MOTOR_RIGHT, MotorType.kBrushless);
    m_turret = new CANSparkMax(RobotMap.A_TURRET_MOTOR, MotorType.kBrushless);
    m_wrist = new CANSparkMax(RobotMap.A_WRIST_MOTOR, MotorType.kBrushless);

    // Acquire references to ARM encoders
    m_shoulderEncoderLeft = m_shoulderLeft.getEncoder();
    m_shoulderEncoderRight = m_shoulderRight.getEncoder();
    m_elbowEncoderLeft = m_elbowLeft.getEncoder();
    m_elbowEncoderRight = m_elbowRight.getEncoder();
    m_turretEncoder = m_turret.getEncoder();
    m_wristEncoder = m_wrist.getEncoder();

    // Initialize ARM potentiometers
    m_shoulderPot = new AnalogInput(RobotMap.A_SHOULDER_POTENTIOMETER);
    m_elbowPot = new AnalogInput(RobotMap.A_ELBOW_POTENTIOMETER);
    m_turretPot = new AnalogInput(RobotMap.A_TURRET_POTENTIOMETER);

    // Set inversion for the elbow and shoulder
    m_shoulderLeft.setInverted(true);
    m_elbowLeft.setInverted(true);
    m_shoulderRight.setInverted(false);
    m_shoulderRight.setInverted(false);

    // Setup the LEFT shoulder/elbow to follow
    // the RIGHT shoulder/elbow
    m_shoulderLeft.follow(m_shoulderRight);
    m_elbowLeft.follow(m_shoulderRight);

    // The shoulder and elbow motors will be driven by
    // working through their RIGHT sides
    m_shoulder = m_shoulderRight;
    m_elbow = m_elbowRight;

  }

  public static Arm getInstance() {
    if (m_arm == null) {
      m_arm = new Arm();
      TestingDashboard.getInstance().registerSubsystem(m_arm, "Arm");
      TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "ElbowPotVoltage", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "ShoulderPotVoltage", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "TurretPotVoltage", 0);

      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "ElbowEncoderLeftPulses", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "ElbowEncoderRightPulses", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "ShoulderEncoderLeftPulses", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "ShoulderEncoderRightPulses", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "TurretEncoderPulses", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "Encoders", "WristEncoderPulses", 0);

      TestingDashboard.getInstance().registerNumber(m_arm, "MotorInputs", "ElbowMotorPower", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "MotorInputs", "ShoulderMotorPower", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "MotorInputs", "TurretMotorPower", 0);
      TestingDashboard.getInstance().registerNumber(m_arm, "MotorInputs", "WristMotorPower", 0);
    }
    return m_arm;
  }

  public double getTurretAngle() {
    double turretEncoderAngle = m_turretEncoder.getPosition() * Constants.TURRET_DEGREES_PER_PULSE;
    double turretPotAngle = m_turretPot.getVoltage() * Constants.TURRET_POT_DEGREES_PER_VOLT;
    // TODO: Compare and check if they match
    return turretPotAngle;
  }

  public double getShoulderAngle() {
    double shoulderEncoderLeftAngle = m_shoulderEncoderLeft.getPosition() * Constants.SHOULDER_DEGREES_PER_PULSE;
    double shoulderEncoderRightAngle = m_shoulderEncoderRight.getPosition() * Constants.SHOULDER_DEGREES_PER_PULSE;
    double shoulderPotAngle = m_shoulderPot.getVoltage() * Constants.SHOULDER_POT_DEGREES_PER_VOLT;
    // TODO: Compare all 3 and discard 1 if it doesn't match
    return shoulderPotAngle;
  }

  public double getElbowAngle() {
    double elbowEncoderLeftAngle = m_elbowEncoderLeft.getPosition() * Constants.ELBOW_DEGREES_PER_PULSE;
    double elbowEncoderRightAngle = m_elbowEncoderRight.getPosition() * Constants.ELBOW_DEGREES_PER_PULSE;
    double elbowPotAngle = m_elbowPot.getVoltage() * Constants.ELBOW_POT_DEGREES_PER_VOLT;
    // TODO: Compare all 3 and discard 1 if it doesn't match
    return elbowPotAngle;
  }

  public double getWristAngle() {
    double wristEncoderAngle = m_wristEncoder.getPosition() * Constants.WRIST_DEGREES_PER_PULSE;
    return wristEncoderAngle;
  }

  public void setTurretMotorPower(double value) {
    m_turret.set(value);
  }

  public void setShoulderMotorPower(double value) {
    m_shoulder.set(value);
  }

  public void setElbowMotorPower(double value) {
    m_elbow.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowPotVoltage", m_elbowPot.getVoltage());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderPotVoltage", m_shoulderPot.getVoltage());
    TestingDashboard.getInstance().updateNumber(m_arm, "TurretPotVoltage", m_turretPot.getVoltage());

    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowEncoderLeftPulses", m_elbowEncoderLeft.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowEncoderRightPulses", m_elbowEncoderRight.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderEncoderLeftPulses", m_shoulderEncoderLeft.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderEncoderRightPulses", m_shoulderEncoderRight.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "TurretEncoderPulses", m_turretEncoder.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "WristEncoderPulses", m_wristEncoder.getPosition());
  }
}
