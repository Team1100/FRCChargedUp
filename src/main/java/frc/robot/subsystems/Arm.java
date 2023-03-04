// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Vector;
import frc.robot.helpers.ArmSegmentHelper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TestingDashboard;

import java.awt.geom.Point2D;

import org.opencv.features2d.KAZE;

public class Arm extends SubsystemBase {

  private static Arm m_arm;
  private ArmSegmentHelper m_armHelper;

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

  private AnalogInput m_shoulderPotLeft;
  private AnalogInput m_elbowPotLeft;
  private AnalogInput m_shoulderPotRight;
  private AnalogInput m_elbowPotRight;
  private AnalogInput m_turretPot;

  // PID controllers and enable/disable
  private boolean m_enableArmPid = false;
  private PIDController m_shoulderPid;
  private boolean m_enableShoulderPid = false;
  private PIDController m_elbowPid;
  private boolean m_enableElbowPid = false;
  private PIDController m_turretPid;
  private boolean m_enableTurretPid = false;
  private PIDController m_wristPid;
  private boolean m_enableWristPid = false;

  private double m_shoulderTargetAngle;
  private double m_elbowTargetAngle;
  private double m_turretTargetAngle;
  private double m_wristTargetAngle;

  private boolean shoulderEnabled;
  private boolean elbowEnabled;
  private boolean turretEnabled;
  private boolean wristEnabled;

  // Determines whether or not the motors will shut off if they pull too much current.
  private final static boolean CURRENT_LIMITING_ENABLED = true;
  
  public final static int DEFAULT_MOTOR_CURRENT_LIMITS = 65;



  /** Creates a new Arm. */
  private Arm() {

    // Initialize ARM motors
    m_shoulderLeft = new CANSparkMax(RobotMap.A_SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
    m_shoulderRight = new CANSparkMax(RobotMap.A_SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
    m_elbowLeft = new CANSparkMax(RobotMap.A_ELBOW_MOTOR_LEFT, MotorType.kBrushless);
    m_elbowRight = new CANSparkMax(RobotMap.A_ELBOW_MOTOR_RIGHT, MotorType.kBrushless);
    m_turret = new CANSparkMax(RobotMap.A_TURRET_MOTOR, MotorType.kBrushless);
    m_wrist = new CANSparkMax(RobotMap.A_WRIST_MOTOR, MotorType.kBrushless);

    m_shoulderLeft.setSmartCurrentLimit(DEFAULT_MOTOR_CURRENT_LIMITS);
    m_shoulderRight.setSmartCurrentLimit(DEFAULT_MOTOR_CURRENT_LIMITS);
    m_elbowLeft.setSmartCurrentLimit(DEFAULT_MOTOR_CURRENT_LIMITS);
    m_elbowRight.setSmartCurrentLimit(DEFAULT_MOTOR_CURRENT_LIMITS);
    m_turret.setSmartCurrentLimit(DEFAULT_MOTOR_CURRENT_LIMITS);
    m_wrist.setSmartCurrentLimit(DEFAULT_MOTOR_CURRENT_LIMITS);

    m_shoulderLeft.restoreFactoryDefaults();
    m_shoulderRight.restoreFactoryDefaults();
    m_elbowLeft.restoreFactoryDefaults();
    m_elbowRight.restoreFactoryDefaults();
    m_turret.restoreFactoryDefaults();
    m_wrist.restoreFactoryDefaults();

    // Acquire references to ARM encoders
    m_shoulderEncoderLeft = m_shoulderLeft.getEncoder();
    m_shoulderEncoderRight = m_shoulderRight.getEncoder();
    m_elbowEncoderLeft = m_elbowLeft.getEncoder();
    m_elbowEncoderRight = m_elbowRight.getEncoder();
    m_turretEncoder = m_turret.getEncoder();
    m_wristEncoder = m_wrist.getEncoder();

    // Initializes an ArmSegmentHelper
    m_armHelper = new ArmSegmentHelper();

    zeroEncoders();

    // Initialize ARM potentiometers
    m_shoulderPotLeft = new AnalogInput(RobotMap.A_SHOULDER_POTENTIOMETER_LEFT);
    m_elbowPotLeft = new AnalogInput(RobotMap.A_ELBOW_POTENTIOMETER_LEFT);
    m_shoulderPotLeft = new AnalogInput(RobotMap.A_SHOULDER_POTENTIOMETER_RIGHT);
    m_elbowPotLeft = new AnalogInput(RobotMap.A_ELBOW_POTENTIOMETER_RIGHT);
    m_turretPot = new AnalogInput(RobotMap.A_TURRET_POTENTIOMETER);

    // Sets arm motors to brake mode
    m_shoulderLeft.setIdleMode(IdleMode.kBrake);
    m_shoulderRight.setIdleMode(IdleMode.kBrake);
    m_elbowLeft.setIdleMode(IdleMode.kBrake);
    m_elbowRight.setIdleMode(IdleMode.kBrake);
    m_turret.setIdleMode(IdleMode.kBrake);
    m_wrist.setIdleMode(IdleMode.kBrake);

    // Set inversion for the elbow and shoulder
    m_shoulderRight.setInverted(false);
    m_elbowRight.setInverted(false);

    // Setup the LEFT shoulder/elbow to follow
    // the RIGHT shoulder/elbow
    m_shoulderLeft.follow(m_shoulderRight, true);
    m_elbowLeft.follow(m_elbowRight, true);

    // The shoulder and elbow motors will be driven by
    // working through their RIGHT sides
    m_shoulder = m_shoulderRight;
    m_elbow = m_elbowRight;

    shoulderEnabled = true;
    elbowEnabled = true;
    turretEnabled = true;
    wristEnabled = true;

    if (Constants.A_ENABLE_SOFTWARE_PID) {
      m_shoulderPid = new PIDController(Constants.A_SHOULDER_SOFTWARE_P, Constants.A_SHOULDER_SOFTWARE_I, Constants.A_SHOULDER_SOFTWARE_D);
      m_elbowPid = new PIDController(Constants.A_ELBOW_SOFTWARE_P, Constants.A_ELBOW_SOFTWARE_I, Constants.A_ELBOW_SOFTWARE_D);
      m_turretPid = new PIDController(Constants.A_TURRET_SOFTWARE_P, Constants.A_TURRET_SOFTWARE_I, Constants.A_TURRET_SOFTWARE_D);
      m_wristPid = new PIDController(Constants.A_WRIST_SOFTWARE_P, Constants.A_WRIST_SOFTWARE_I, Constants.A_WRIST_SOFTWARE_D);
    }

    initializeJointTargetAngles();

  }

  public static Arm getInstance() {
			if (m_arm == null) {
        m_arm = new Arm();
        TestingDashboard.getInstance().registerSubsystem(m_arm, "Arm");
        TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "ElbowPotLeftVoltage", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "ShoulderPotLeftVoltage", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "ElbowPotRightVoltage", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "Potentiometers", "ShoulderPotRightVoltage", 0);
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

        TestingDashboard.getInstance().registerString(m_arm, "PidMasterControl", "ArmSoftwarePidEnable", "Disabled");

        TestingDashboard.getInstance().registerString(m_arm, "PidJointControl", "WristSoftwarePidEnable", "Disabled");
        TestingDashboard.getInstance().registerString(m_arm, "PidJointControl", "ElbowSoftwarePidEnable", "Disabled");
        TestingDashboard.getInstance().registerString(m_arm, "PidJointControl", "ShoulderSoftwarePidEnable", "Disabled");
        TestingDashboard.getInstance().registerString(m_arm, "PidJointControl", "TurretSoftwarePidEnable", "Disabled");


        TestingDashboard.getInstance().registerNumber(m_arm, "TurretSoftwarePID", "TargetTurretAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "TurretSoftwarePID", "TargetTurretTolerance", Constants.A_TURRET_SOFTWARE_TOLERANCE);
        TestingDashboard.getInstance().registerNumber(m_arm, "TurretSoftwarePID", "TargetTurretP", Constants.A_TURRET_SOFTWARE_P);
        TestingDashboard.getInstance().registerNumber(m_arm, "TurretSoftwarePID", "TargetTurretI", Constants.A_TURRET_SOFTWARE_I);
        TestingDashboard.getInstance().registerNumber(m_arm, "TurretSoftwarePID", "TargetTurretD", Constants.A_TURRET_SOFTWARE_D);

        TestingDashboard.getInstance().registerNumber(m_arm, "ShoulderSoftwarePID", "TargetShoulderAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "ShoulderSoftwarePID", "TargetShoulderTolerance", Constants.A_SHOULDER_SOFTWARE_TOLERANCE);
        TestingDashboard.getInstance().registerNumber(m_arm, "ShoulderSoftwarePID", "TargetShoulderP", Constants.A_SHOULDER_SOFTWARE_P);
        TestingDashboard.getInstance().registerNumber(m_arm, "ShoulderSoftwarePID", "TargetShoulderI", Constants.A_SHOULDER_SOFTWARE_I);
        TestingDashboard.getInstance().registerNumber(m_arm, "ShoulderSoftwarePID", "TargetShoulderD", Constants.A_SHOULDER_SOFTWARE_D);

        TestingDashboard.getInstance().registerNumber(m_arm, "ElbowSoftwarePID", "TargetElbowAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "ElbowSoftwarePID", "TargetElbowTolerance", Constants.A_ELBOW_SOFTWARE_TOLERANCE);
        TestingDashboard.getInstance().registerNumber(m_arm, "ElbowSoftwarePID", "TargetElbowP", Constants.A_ELBOW_SOFTWARE_P);
        TestingDashboard.getInstance().registerNumber(m_arm, "ElbowSoftwarePID", "TargetElbowI", Constants.A_ELBOW_SOFTWARE_I);
        TestingDashboard.getInstance().registerNumber(m_arm, "ElbowSoftwarePID", "TargetElbowD", Constants.A_ELBOW_SOFTWARE_D);

        TestingDashboard.getInstance().registerNumber(m_arm, "WristSoftwarePID", "TargetWristAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "WristSoftwarePID", "TargetWristTolerance", Constants.A_WRIST_SOFTWARE_TOLERANCE);
        TestingDashboard.getInstance().registerNumber(m_arm, "WristSoftwarePID", "TargetWristP", Constants.A_WRIST_SOFTWARE_P);
        TestingDashboard.getInstance().registerNumber(m_arm, "WristSoftwarePID", "TargetWristI", Constants.A_WRIST_SOFTWARE_I);
        TestingDashboard.getInstance().registerNumber(m_arm, "WristSoftwarePID", "TargetWristD", Constants.A_WRIST_SOFTWARE_D);

        TestingDashboard.getInstance().registerNumber(m_arm, "HandCoordinates", "HandXCoor", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "HandCoordinates", "HandYCoor", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "HandCoordinates", "HandZCoor", 0);

        TestingDashboard.getInstance().registerNumber(m_arm, "HandVelocity", "HandXVel", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "HandVelocity", "HandYVel", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "HandVelocity", "HandZVel", 0);

        TestingDashboard.getInstance().registerNumber(m_arm, "JointAnglesAll", "TurretPotAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "JointAnglesAll", "TurretEncoderAngle", 0);

        TestingDashboard.getInstance().registerNumber(m_arm, "JointAnglesAll", "ShoulderPotAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "JointAnglesAll", "ShoulderEncoderLeftAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "JointAnglesAll", "ShoulderEncoderRightAngle", 0);

        TestingDashboard.getInstance().registerNumber(m_arm, "JointAnglesAll", "ElbowPotAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "JointAnglesAll", "ElbowEncoderLeftAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "JointAnglesAll", "ElbowEncoderRightAngle", 0);

        TestingDashboard.getInstance().registerNumber(m_arm, "JointAngles", "TurretAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "JointAngles", "ShoulderAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "JointAngles", "ElbowAngle", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "JointAngles", "WristAngle", 0);

        TestingDashboard.getInstance().registerNumber(m_arm, "MotorCurrents", "TurretCurrent", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "MotorCurrents", "ShoulderCurrent", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "MotorCurrents", "ElbowCurrent", 0);
        TestingDashboard.getInstance().registerNumber(m_arm, "MotorCurrents", "WristCurrent", 0);
    }
    return m_arm;
  }

  public void initializeJointTargetAngles() {
    // TODO: Define constants for joint starting angles
    m_shoulderTargetAngle = 0;
    m_elbowTargetAngle = 0;
    m_turretTargetAngle = 0;
    m_wristTargetAngle = 0;
  }

  public void setWristTargetAngle(double angle) {
    m_wristTargetAngle = angle;
  }

  public void setElbowTargetAngle(double angle) {
    m_elbowTargetAngle = angle;
  }

  public void setShoulderTargetAngle(double angle) {
    m_shoulderTargetAngle = angle;
  }

  public void setTurretTargetAngle(double angle) {
    m_turretTargetAngle = angle;
  }

  public double getWristTargetAngle() {
    return m_wristTargetAngle;
  }

  public double getElbowTargetAngle() {
    return m_elbowTargetAngle;
  }

  public double getShoulderTargetAngle() {
    return m_shoulderTargetAngle;
  }

  public double getTurretTargetAngle() {
    return m_turretTargetAngle;
  }

  private double getTurretPotAngle() {
    return m_turretPot.getVoltage() * Constants.TURRET_POT_DEGREES_PER_VOLT;
  }

  private double getTurretEncoderAngle() {
    return m_turretEncoder.getPosition() * Constants.TURRET_DEGREES_PER_PULSE;
  }

  public double getTurretAngle() {
    double turretEncoderAngle = getTurretEncoderAngle();
    double turretPotAngle = getTurretPotAngle();
    // TODO: Compare and check if they match
    return turretEncoderAngle;
  }

  private double getShoulderEncoderLeftAngle() {
    return m_shoulderEncoderLeft.getPosition() * Constants.SHOULDER_DEGREES_PER_PULSE;
  }

  private double getShoulderEncoderRightAngle() {
    return m_shoulderEncoderRight.getPosition() * Constants.SHOULDER_DEGREES_PER_PULSE;
  }

  private double getShoulderPotLeftAngle() {
    return m_shoulderPotLeft.getVoltage() * Constants.SHOULDER_POT_LEFT_DEGREES_PER_VOLT;
  }

  private double getShoulderPotRightAngle() {
    return m_shoulderPotRight.getVoltage() * Constants.SHOULDER_POT_RIGHT_DEGREES_PER_VOLT;
  }

  public double getShoulderAngle() {
    double shoulderEncoderLeftAngle = getShoulderEncoderLeftAngle();
    double shoulderEncoderRightAngle = getShoulderEncoderRightAngle();
    double shoulderPotLeftAngle = getShoulderPotLeftAngle();
    double shoulderPotRightAngle = getShoulderPotRightAngle();
    // TODO: Compare all 3 and discard 1 if it doesn't match
    return shoulderEncoderLeftAngle;
  }

  private double getElbowEncoderLeftAngle() {
    return m_elbowEncoderLeft.getPosition() * Constants.ELBOW_DEGREES_PER_PULSE;
  }

  private double getElbowEncoderRightAngle() {
    return m_elbowEncoderRight.getPosition() * Constants.ELBOW_DEGREES_PER_PULSE;
  }

  private double getElbowPotLeftAngle() {
    return m_elbowPotLeft.getVoltage() * Constants.ELBOW_POT_LEFT_DEGREES_PER_VOLT;
  }

  private double getElbowPotRightAngle() {
    return m_elbowPotRight.getVoltage() * Constants.ELBOW_POT_RIGHT_DEGREES_PER_VOLT;
  }

  public double getElbowAngle() {
    double elbowEncoderLeftAngle = getElbowEncoderLeftAngle();
    double elbowEncoderRightAngle = getElbowEncoderRightAngle();
    double elbowPotLeftAngle = getElbowPotLeftAngle();
    double elbowPotRightAngle = getElbowPotRightAngle();
    // TODO: Compare all 3 and discard 1 if it doesn't match
    return elbowEncoderLeftAngle;
  }

  public double getWristAngle() {
    double wristEncoderAngle = m_wristEncoder.getPosition() * Constants.WRIST_DEGREES_PER_PULSE;
    return wristEncoderAngle;
  }

  public double getTurretVelocity() {
    return m_turretEncoder.getVelocity();
  }

  public double getShoulderVelocity() {
    return (m_shoulderEncoderLeft.getVelocity() + m_shoulderEncoderRight.getVelocity())/2;
  }

  public double getElbowVelocity() {
    return (m_elbowEncoderLeft.getVelocity() + m_elbowEncoderRight.getVelocity())/2;
  }

  public void setTurretMotorPower(double value) {
    if (turretEnabled) {
      m_turret.set(value);
    }
    
  }

  public void setShoulderMotorPower(double value) {
    if (shoulderEnabled) {
    m_shoulder.set(value);
    }
  }

  public void setElbowMotorPower(double value) {
    if (elbowEnabled) {
    m_elbow.set(value);
    }
  }

  public void setWristMotorPower(double value) {
    if (wristEnabled) {
    m_wrist.set(value);
    }
  }

  public void toggleTurretPower(boolean enabled) {
    wristEnabled = enabled;
  }

  public void toggleShoulderPower(boolean enabled) {
    shoulderEnabled = enabled;
  }

  public void toggleElbowPower(boolean enabled) {
    elbowEnabled = enabled;
  }

  public void toggleWristPower(boolean enabled) {
    wristEnabled = enabled;
  }

  public void zeroEncoders() {
    m_shoulderEncoderLeft.setPosition(0);
    m_shoulderEncoderRight.setPosition(0);
    m_elbowEncoderLeft.setPosition(0);
    m_elbowEncoderRight.setPosition(0);
    m_turretEncoder.setPosition(0);
    m_wristEncoder.setPosition(0);
  }

  public RelativeEncoder getShoulderEncoderLeft() {
    return m_shoulderEncoderLeft;
  }

  public RelativeEncoder getShoulderEncoderRight() {
    return m_shoulderEncoderRight;
  }

  public RelativeEncoder getElbowEncoderLeft() {
    return m_elbowEncoderLeft;
  }

  public RelativeEncoder getElbowEncoderRight() {
    return m_elbowEncoderRight;
  }

  public RelativeEncoder getTurretEncoder() {
    return m_turretEncoder;
  }

  public RelativeEncoder getWristEncoder() {
    return m_wristEncoder;
  }

  public void enableAllMotors() {
    toggleTurretPower(true);
    toggleShoulderPower(true);
    toggleElbowPower(true);
    toggleWristPower(true);
  }

  public void disableAllMotors() {
    toggleTurretPower(false);
    toggleShoulderPower(false);
    toggleElbowPower(false);
    toggleWristPower(false);
  }

  public double getHandX(/*double theta1, double theta2, double rotation*/) {
    double x = 0;

    double thetaOne = getShoulderAngle();
    double thetaTwo = getElbowAngle();
    double rotation = getTurretAngle();

    double theta1 = thetaOne;
    double theta2 = thetaTwo;
    double rot = rotation;

    x = (41.5 * Math.sin(Math.toRadians(theta1))) - (32 * Math.sin(Math.toRadians(theta2)));

    return x;
  }

  public double getHandY(/*double thetaOne, double thetaTwo, double rotation*/) {
    double y = 0;

    double thetaOne = getShoulderAngle();
    double thetaTwo = getElbowAngle();
    double rotation = getTurretAngle();

    double theta1 = thetaOne;
    double theta2 = thetaTwo;
    double rot = rotation;

    y = (41.5 * Math.cos(Math.toRadians(theta1))) - (32 * Math.cos(Math.toRadians(theta2)));

    return y;
  }

  public void enableArmPid() {
    m_enableArmPid = true;
    enableWristPid();
    enableElbowPid();
    enableShoulderPid();
    enableTurretPid();
  }

  public void disableArmPid() {
    m_enableArmPid = false;
    disableWristPid();
    disableElbowPid();
    disableShoulderPid();
    disableTurretPid();
  }

  public void enableWristPid() {
    m_enableArmPid = true;
    m_enableWristPid = true;
  }

  public void disableWristPid() {
    m_enableWristPid = false;
    setWristMotorPower(0.0d);
  }

  public void enableElbowPid() {
    m_enableArmPid = true;
    m_enableElbowPid = true;
  }

  public void disableElbowPid() {
    m_enableElbowPid = false;
    setElbowMotorPower(0.0d);
  }

  public void enableShoulderPid() {
    m_enableArmPid = true;
    m_enableShoulderPid = true;
  }

  public void disableShoulderPid() {
    m_enableShoulderPid = false;
    setShoulderMotorPower(0.0d);
  }

  public void enableTurretPid() {
    m_enableArmPid = true;
    m_enableTurretPid = true;
  }

  public void disableTurretPid() {
    m_enableTurretPid = false;
    setTurretMotorPower(0.0d);
  }

  public void checkTurretCurrentOutput() {
    if (m_turret.getOutputCurrent() > Constants.TURRET_MOTOR_CURRENT_LIMIT) {
      m_turret.set(0);
      turretEnabled = false;
    }
  }

  public void checkShoulderCurrentOutput() {
    boolean left = m_shoulderLeft.getOutputCurrent() > Constants.SHOULDER_MOTOR_CURRENT_LIMIT;
    boolean right = m_shoulderRight.getOutputCurrent() > Constants.SHOULDER_MOTOR_CURRENT_LIMIT;
    if (left || right) {
      m_shoulder.set(0);
      shoulderEnabled = false;
    }
  }

  public void checkElbowCurrentOutput() {
    boolean left = m_elbowLeft.getOutputCurrent() > Constants.ELBOW_MOTOR_CURRENT_LIMIT;
    boolean right = m_elbowRight.getOutputCurrent() > Constants.ELBOW_MOTOR_CURRENT_LIMIT;
    if (left || right) {
      m_elbow.set(0);
      elbowEnabled = false;
    }
  }

  public void checkWristCurrentOutput() {
    if (m_wrist.getOutputCurrent() > Constants.WRIST_MOTOR_CURRENT_LIMIT) {
      m_wrist.set(0);
      wristEnabled = false;
    }
  }



  public void updateJointSoftwarePidControllerValues() {
    double p, i, d, tolerance;
    p = TestingDashboard.getInstance().getNumber(m_arm, "TargetTurretP");
    i = TestingDashboard.getInstance().getNumber(m_arm, "TargetTurretI");
    d = TestingDashboard.getInstance().getNumber(m_arm, "TargetTurretD");
    tolerance = TestingDashboard.getInstance().getNumber(m_arm, "TargetTurretTolerance");
    m_turretPid.setP(p);
    m_turretPid.setI(i);
    m_turretPid.setD(d);
    m_turretPid.setTolerance(tolerance);
    m_turretPid.setSetpoint(m_turretTargetAngle);
    
    p = TestingDashboard.getInstance().getNumber(m_arm, "TargetShoulderP");
    i = TestingDashboard.getInstance().getNumber(m_arm, "TargetShoulderI");
    d = TestingDashboard.getInstance().getNumber(m_arm, "TargetShoulderD");
    tolerance = TestingDashboard.getInstance().getNumber(m_arm, "TargetShoulderTolerance");
    m_shoulderPid.setP(p);
    m_shoulderPid.setI(i);
    m_shoulderPid.setD(d);
    m_shoulderPid.setTolerance(tolerance);
    m_shoulderPid.setSetpoint(m_shoulderTargetAngle);

    p = TestingDashboard.getInstance().getNumber(m_arm, "TargetElbowP");
    i = TestingDashboard.getInstance().getNumber(m_arm, "TargetElbowI");
    d = TestingDashboard.getInstance().getNumber(m_arm, "TargetElbowD");
    tolerance = TestingDashboard.getInstance().getNumber(m_arm, "TargetElbowTolerance");
    m_elbowPid.setP(p);
    m_elbowPid.setI(i);
    m_elbowPid.setD(d);
    m_elbowPid.setTolerance(tolerance);
    m_elbowPid.setSetpoint(m_elbowTargetAngle-getShoulderAngle());

    p = TestingDashboard.getInstance().getNumber(m_arm, "TargetWristP");
    i = TestingDashboard.getInstance().getNumber(m_arm, "TargetWristI");
    d = TestingDashboard.getInstance().getNumber(m_arm, "TargetWristD");
    tolerance = TestingDashboard.getInstance().getNumber(m_arm, "TargetWristTolerance");
    m_wristPid.setP(p);
    m_wristPid.setI(i);
    m_wristPid.setD(d);
    m_wristPid.setTolerance(tolerance);
    m_wristPid.setSetpoint(m_wristTargetAngle);
  }

  public void controlJointsWithSoftwarePidControl() {
    updateJointSoftwarePidControllerValues();

    // Do nothing if Arm PID control is not enabled
    if (!m_enableArmPid) {
      return;
    }

    if (m_enableTurretPid) {
      double t_power = m_turretPid.calculate(getTurretAngle(), m_turretTargetAngle);
      t_power = MathUtil.clamp(t_power, -Constants.A_TURRET_MAX_POWER, Constants.A_TURRET_MAX_POWER);
      setTurretMotorPower(t_power);
    }

    if (m_enableShoulderPid) {
      double s_power = m_shoulderPid.calculate(getShoulderAngle(), m_shoulderTargetAngle);
      s_power = MathUtil.clamp(s_power, -Constants.A_SHOULDER_MAX_POWER, Constants.A_SHOULDER_MAX_POWER);
      setShoulderMotorPower(s_power);
    }

    if (m_enableElbowPid) {
      double e_power = m_elbowPid.calculate(getElbowAngle(), m_elbowTargetAngle);
      e_power = MathUtil.clamp(e_power, -Constants.A_ELBOW_MAX_POWER, Constants.A_ELBOW_MAX_POWER);
      setElbowMotorPower(e_power);
    }

    if (m_enableWristPid) {
      double w_power = m_wristPid.calculate(getWristAngle(), m_wristTargetAngle);
      w_power = MathUtil.clamp(w_power, -Constants.A_WRIST_MAX_POWER, Constants.A_WRIST_MAX_POWER);
      setWristMotorPower(w_power);
    }
  }

  public void updatePidEnableFlags() {
    if (m_enableArmPid) {
      TestingDashboard.getInstance().updateString(m_arm, "ArmSoftwarePidEnable", "Enabled");
    } else {
      TestingDashboard.getInstance().updateString(m_arm, "ArmSoftwarePidEnable", "Disabled");
    }

    if (m_enableWristPid) {
      TestingDashboard.getInstance().updateString(m_arm, "WristSoftwarePidEnable", "Enabled");
    } else {
      TestingDashboard.getInstance().updateString(m_arm, "WristSoftwarePidEnable", "Disabled");
    }

    if (m_enableElbowPid) {
      TestingDashboard.getInstance().updateString(m_arm, "ElbowSoftwarePidEnable", "Enabled");
    } else {
      TestingDashboard.getInstance().updateString(m_arm, "ElbowSoftwarePidEnable", "Disabled");
    }

    if (m_enableShoulderPid) {
      TestingDashboard.getInstance().updateString(m_arm, "ShoulderSoftwarePidEnable", "Enabled");
    } else {
      TestingDashboard.getInstance().updateString(m_arm, "ShoulderSoftwarePidEnable", "Disabled");
    }

    if (m_enableTurretPid) {
      TestingDashboard.getInstance().updateString(m_arm, "TurretSoftwarePidEnable", "Enabled");
    } else {
      TestingDashboard.getInstance().updateString(m_arm, "TurretSoftwarePidEnable", "Disabled");
    }
  }

  public PIDController getTurretPID() {
    return m_turretPid;
  }

  public PIDController getShoulderPID() {
    return m_shoulderPid;
  }

  public PIDController getElbowPID() {
    return m_elbowPid;
  }

  public PIDController getWristPID() {
    return m_wristPid;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowPotLeftVoltage", m_elbowPotLeft.getVoltage());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderPotLeftVoltage", m_shoulderPotLeft.getVoltage());
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowPotRightVoltage", m_elbowPotRight.getVoltage());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderPotRightVoltage", m_shoulderPotRight.getVoltage());
    TestingDashboard.getInstance().updateNumber(m_arm, "TurretPotVoltage", m_turretPot.getVoltage());

    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowEncoderLeftPulses", m_elbowEncoderLeft.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowEncoderRightPulses", m_elbowEncoderRight.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderEncoderLeftPulses", m_shoulderEncoderLeft.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderEncoderRightPulses", m_shoulderEncoderRight.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "TurretEncoderPulses", m_turretEncoder.getPosition());
    TestingDashboard.getInstance().updateNumber(m_arm, "WristEncoderPulses", m_wristEncoder.getPosition());

    TestingDashboard.getInstance().updateNumber(m_arm, "TurretPotAngle", getTurretPotAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "TurretEncoderAngle", getTurretEncoderAngle());

    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderPotLeftAngle", getShoulderPotLeftAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderPotRightAngle", getShoulderPotRightAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderEncoderLeftAngle", getShoulderEncoderLeftAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderEncoderRightAngle", getShoulderEncoderRightAngle());

    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowPotLeftAngle", getElbowPotLeftAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowPotRightAngle", getElbowPotRightAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowEncoderLeftAngle", getElbowEncoderLeftAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowEncoderRightAngle", getElbowEncoderRightAngle());

    TestingDashboard.getInstance().updateNumber(m_arm, "TurretAngle", getTurretAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderAngle", getShoulderAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowAngle", getElbowAngle());
    TestingDashboard.getInstance().updateNumber(m_arm, "WristAngle", getWristAngle());

    TestingDashboard.getInstance().updateNumber(m_arm, "TurretCurrent", m_turret.getOutputCurrent());
    TestingDashboard.getInstance().updateNumber(m_arm, "ShoulderCurrent", m_shoulder.getOutputCurrent());
    TestingDashboard.getInstance().updateNumber(m_arm, "ElbowCurrent", m_elbow.getOutputCurrent());
    TestingDashboard.getInstance().updateNumber(m_arm, "WristCurrent", m_wrist.getOutputCurrent());

    updatePidEnableFlags();

    m_armHelper.updateArmSegmentValues();

    if (Constants.A_ENABLE_SOFTWARE_PID && m_enableArmPid) {
      controlJointsWithSoftwarePidControl();
    }

    if (CURRENT_LIMITING_ENABLED) {
      checkTurretCurrentOutput();
      checkShoulderCurrentOutput();
      checkElbowCurrentOutput();
      checkWristCurrentOutput();
    }


  }

}
