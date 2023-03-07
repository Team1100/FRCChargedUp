// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.Pipe;
import java.util.ArrayList;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.RoboRioAccelerometerHelper;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TestingDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Drive extends SubsystemBase {

  private CANSparkMax m_backLeft;
  private CANSparkMax m_backRight;
  private CANSparkMax m_frontLeft;
  private CANSparkMax m_frontRight;
  private RelativeEncoder m_backLeftEncoder;
  private RelativeEncoder m_backRightEncoder;
  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;
  private SparkMaxPIDController m_leftController;
  private SparkMaxPIDController m_rightController;
  private DifferentialDrive drivetrain;

  private BuiltInAccelerometer m_accelerometer;
  private RoboRioAccelerometerHelper m_accelHelper;
  private boolean m_measureVelocity;
  private boolean m_measureDistance;
  private double accelIntCount = 0;
  private IdleMode m_currentIdleMode;

  private boolean m_currentPIDMode;

  public double fwdRateLimit = Constants.D_FWD_RATE_LIMIT; // limits rate change to a certain amount per second. Measured in units
  public  double rotRateLimit = Constants.D_ROT_RATE_LIMIT;

  // Motor current variables
  ArrayList<Double> m_left_motor_current_values;
  ArrayList<Double> m_right_motor_current_values;
  public static final int MOTOR_CURRENT_INITIAL_CAPACITY = 50; // This is 1000 miliseconds divided in 20 millisecond chunks
  private int m_max_num_current_values;

  double m_rightSpeed;
  double m_leftSpeed;

  public static final double WHEEL_DIAMETER_IN_INCHES = 4; 
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_IN_INCHES * Math.PI;
  public static final double GEAR_RATIO = 6.85; //number of times the motor rotates to rotate wheel once
  public static final double CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE / GEAR_RATIO; //conversion factor * circumference = distance
  public final static double DISTANCE = CONVERSION_FACTOR * WHEEL_CIRCUMFERENCE;
  public final static double INITIAL_SPEED = 0.3;
  
  private static Drive m_drive;

  /** Creates a new Drive. */
  private Drive() {

    m_backLeft = new CANSparkMax(RobotMap.D_BACK_LEFT, MotorType.kBrushless);
    m_backRight = new CANSparkMax(RobotMap.D_BACK_RIGHT, MotorType.kBrushless);
    m_frontLeft = new CANSparkMax(RobotMap.D_FRONT_LEFT, MotorType.kBrushless);
    m_frontRight = new CANSparkMax(RobotMap.D_FRONT_RIGHT, MotorType.kBrushless);

    m_backLeftEncoder = m_backLeft.getEncoder();
    m_backRightEncoder = m_backRight.getEncoder();
    m_frontLeftEncoder = m_frontLeft.getEncoder();
    m_frontRightEncoder = m_frontRight.getEncoder();

    m_leftController = m_frontLeft.getPIDController();
    m_rightController = m_frontRight.getPIDController();

    setupPIDControllers();

    m_currentPIDMode = false;

    m_backLeft.restoreFactoryDefaults();
    m_backRight.restoreFactoryDefaults();
    m_frontLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults(); 

    m_backLeft.setInverted(false);
    m_frontLeft.setInverted(false);
    m_backRight.setInverted(true);
    m_frontRight.setInverted(true);

    m_backLeft.follow(m_frontLeft);
    m_backRight.follow(m_frontRight);

    drivetrain = new DifferentialDrive(m_frontLeft, m_frontRight);

    setIdleMode(IdleMode.kCoast);
    m_currentIdleMode = IdleMode.kCoast;
    setEncoderConversionFactor(CONVERSION_FACTOR);

    if(Constants.D_ENABLE_RAMP_RATE) {
    m_backLeft.setOpenLoopRampRate(Constants.DRIVE_RAMP_RATE);
    m_frontLeft.setOpenLoopRampRate(Constants.DRIVE_RAMP_RATE);
    m_backRight.setOpenLoopRampRate(Constants.DRIVE_RAMP_RATE);
    m_frontRight.setOpenLoopRampRate(Constants.DRIVE_RAMP_RATE);
    }

    m_accelerometer = new BuiltInAccelerometer(); // unit: g
    m_accelHelper = new RoboRioAccelerometerHelper(m_accelerometer);

    m_measureVelocity = false;
    m_measureDistance = false;

    // initialize motor current variables
    m_left_motor_current_values = new ArrayList<Double>(MOTOR_CURRENT_INITIAL_CAPACITY);
    for (int i = 0; i < MOTOR_CURRENT_INITIAL_CAPACITY; i++) {
      m_left_motor_current_values.add(0.0);
    }
    m_right_motor_current_values = new ArrayList<Double>(MOTOR_CURRENT_INITIAL_CAPACITY);
    for (int i = 0; i < MOTOR_CURRENT_INITIAL_CAPACITY; i++) {
      m_right_motor_current_values.add(0.0);
    }

    m_max_num_current_values = MOTOR_CURRENT_INITIAL_CAPACITY;

    SmartDashboard.putNumber("FWD Accel Limit", 3);
    SmartDashboard.putNumber("ROT Accel Limit", 3);

  }

  public void setIdleMode(IdleMode mode) {
    if(m_backLeft.setIdleMode(mode) != REVLibError.kOk){
      System.out.println("Could not set idle mode on back left motor");
      System.exit(1);
    }
  
    if(m_backRight.setIdleMode(mode) != REVLibError.kOk){
      System.out.println("Could not set idle mode on back right motor");
      System.exit(1);
    }
    
    if(m_frontLeft.setIdleMode(mode) != REVLibError.kOk){
      System.out.println("Could not set idle mode on front left motor");
      System.exit(1);
    }
  
    if(m_frontRight.setIdleMode(mode) != REVLibError.kOk){
      System.out.println("Could not set idle mode on front right motor");
      System.exit(1);
    }
  }

  public void setEncoderConversionFactor(double conversionFactor) {
    if(m_backLeftEncoder.setPositionConversionFactor(conversionFactor) != REVLibError.kOk){ 
      System.out.println("Could not set position conversion factor on back left encoder");
    }
  
    if(m_backRightEncoder.setPositionConversionFactor(conversionFactor) != REVLibError.kOk){
      System.out.println("Could not set position conversion factor on back right encoder");
    } 

    if(m_frontLeftEncoder.setPositionConversionFactor(conversionFactor) != REVLibError.kOk){ 
      System.out.println("Could not set position conversion factor on front left encoder");
    }
  
    if(m_frontRightEncoder.setPositionConversionFactor(conversionFactor) != REVLibError.kOk){
      System.out.println("Could not set position conversion factor on front right encoder");
    } 
  }

  public void setupPIDControllers(){
    double leftP, leftI, leftD, leftFF, rightP, rightI, rightD, rightFF, maxOutput, minOutput;

    leftP = TestingDashboard.getInstance().getNumber(m_drive, "LeftP");
    leftI = TestingDashboard.getInstance().getNumber(m_drive, "LeftI");
    leftD = TestingDashboard.getInstance().getNumber(m_drive, "LeftD");
    leftFF = TestingDashboard.getInstance().getNumber(m_drive, "LeftFF");

    rightP = TestingDashboard.getInstance().getNumber(m_drive, "RightP");
    rightI = TestingDashboard.getInstance().getNumber(m_drive, "RightI");
    rightD = TestingDashboard.getInstance().getNumber(m_drive, "RightD");
    rightFF = TestingDashboard.getInstance().getNumber(m_drive, "RightFF");

    maxOutput = TestingDashboard.getInstance().getNumber(m_drive, "PIDOutputMax");
    minOutput = TestingDashboard.getInstance().getNumber(m_drive, "PIDOutputMin");

    m_leftController.setP(leftP);
    m_leftController.setI(leftI);
    m_leftController.setD(leftD);
    m_leftController.setFF(leftFF);

    m_rightController.setP(rightP);
    m_rightController.setI(rightI);
    m_rightController.setD(rightD);
    m_rightController.setFF(rightFF);

    m_leftController.setOutputRange(minOutput, maxOutput);
    m_rightController.setOutputRange(minOutput, maxOutput);
  }

  public static Drive getInstance() {
    if (m_drive == null) {
      m_drive = new Drive();
      TestingDashboard.getInstance().registerSubsystem(m_drive, "Drive");
      TestingDashboard.getInstance().registerNumber(m_drive, "Output", "InitialAngle", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Input", "SpeedWhenTurning", 0.3);
      TestingDashboard.getInstance().registerNumber(m_drive, "Input", "TurnAngleInDegrees", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "BackLeftMotorDistance", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "BackRightMotorDistance", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "FrontLeftMotorDistance", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "FrontRightMotorDistance", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "BackLeftMotorSpeed", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "BackRightMotorSpeed", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "FrontLeftMotorSpeed", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Encoders", "FrontRightMotorSpeed", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "xInstantAccel", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "yInstantAccel", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "instantAccelMagnitudeInchesPerSecondSquared", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "xInstantAccelRaw", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "yInstantAccelRaw", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "xInstantVel", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "yInstantVel", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "xInstantDist", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "yInstantDist", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Accelerometer", "currentTime", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Motors", "BackLeftMotorCurrent", 0); // in Amps
      TestingDashboard.getInstance().registerNumber(m_drive, "Motors", "BackRightMotorCurrent", 0); // in Amps
      TestingDashboard.getInstance().registerNumber(m_drive, "Motors", "FrontLeftMotorCurrent", 0); // in Amps
      TestingDashboard.getInstance().registerNumber(m_drive, "Motors", "FrontRightMotorCurrent", 0); // in Amps
      TestingDashboard.getInstance().registerNumber(m_drive, "Robot", "BatteryVoltage", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Travel", "DistanceToTravelInInches", 12);
      TestingDashboard.getInstance().registerNumber(m_drive, "Travel", "SpeedToTravel", INITIAL_SPEED);
      TestingDashboard.getInstance().registerNumber(m_drive, "Travel", "SpeedOfTravel", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "MotorCurrent", "MaxNumCurrentValues", MOTOR_CURRENT_INITIAL_CAPACITY);
      TestingDashboard.getInstance().registerNumber(m_drive, "Motors", "FrontLeftMotorCurrentAverage", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "Motors", "FrontRightMotorCurrentAverage", 0);
      TestingDashboard.getInstance().registerString(m_drive, "Robot", "DriveIdleMode", "Coast");
      TestingDashboard.getInstance().registerNumber(m_drive, "Motors", "RotCurrentFilteringLimit", Constants.D_ROT_RATE_LIMIT);
      TestingDashboard.getInstance().registerNumber(m_drive, "Motors", "FwdCurrentFilteringLimit", Constants.D_FWD_RATE_LIMIT);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "RightP", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "RightI", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "RightD", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "RightFF", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "LeftP", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "LeftI", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "LeftD", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "LeftFF", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "PIDOutputMax", 0);
      TestingDashboard.getInstance().registerNumber(m_drive, "PIDValues", "PIDOutputMin", 0);
    }
    return m_drive;
  }

  public void toggleIdleMode() {
    if (m_currentIdleMode == IdleMode.kCoast) {
      setIdleMode(IdleMode.kBrake);
      m_currentIdleMode = IdleMode.kBrake;
      TestingDashboard.getInstance().updateString(m_drive, "DriveIdleMode", "Brake");
    } else if (m_currentIdleMode == IdleMode.kBrake) {
      setIdleMode(IdleMode.kCoast);
      m_currentIdleMode = IdleMode.kCoast;
      TestingDashboard.getInstance().updateString(m_drive, "DriveIdleMode", "Coast");
    }
  }

  //Drive Methods:

  public static double integrate(double tInitial, double tFinal, double vInitial, double vFinal) { // v for value
    double tInterval = tFinal - tInitial;
    double area = (tInterval * (vInitial + vFinal)) / 2;
    return area;
  }

  public void startMeasuringVelocity() {
    m_measureVelocity = true;
  }

  
  public void stopMeasuringVelocity() {
    m_measureVelocity = false;
  }

  
  public void startMeasuringDistance() {
    m_measureDistance = true;
  }

  
  public void stopMeasuringDistance() {
    m_measureDistance = false;
  }

  public void setInitialVelocity(double velocity) {
    m_accelHelper.initializeVelocity(velocity);
  }

  public void setInitialDistance(double distance) {
    m_accelHelper.initializeDistance(distance);
  }

  public void arcadeDrive(double fwd, double rot, boolean sqInputs) {
    drivetrain.arcadeDrive(fwd, -rot);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_rightSpeed = rightSpeed;
    m_leftSpeed = leftSpeed;
    drivetrain.tankDrive(m_leftSpeed, m_rightSpeed);
    TestingDashboard.getInstance().updateNumber(m_drive, "SpeedOfTravel", leftSpeed);
  }

  public void setPIDVelocity(double leftRPM, double rightRPM)
  {
    m_leftController.setReference(leftRPM, ControlType.kVelocity);
    m_rightController.setReference(rightRPM, ControlType.kVelocity);
  }

  //Encoder Methods
  public RelativeEncoder getLeftEncoder() {
	  return m_frontLeftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
	  return m_frontRightEncoder;
  }

  public double getTotalAverageLeftMotorCurrent() {
    return arrayListAverage(m_left_motor_current_values);
  }

  public double getTotalAverageRightMotorCurrent() {
    return arrayListAverage(m_right_motor_current_values);
  }

  void updateMotorCurrentAverages() {
    m_max_num_current_values = (int)TestingDashboard.getInstance().getNumber(m_drive, "MaxNumCurrentValues");
    double backLeftMotorCurrent = m_backLeft.getOutputCurrent();
    double backRightMotorCurrent = m_backRight.getOutputCurrent();
    double frontLeftMotorCurrent = m_frontLeft.getOutputCurrent();
    double frontRightMotorCurrent = m_frontRight.getOutputCurrent();
    m_left_motor_current_values.add(backLeftMotorCurrent + frontLeftMotorCurrent);
    m_right_motor_current_values.add(backRightMotorCurrent + frontRightMotorCurrent);
    TestingDashboard.getInstance().updateNumber(m_drive, "BackLeftMotorCurrent", backLeftMotorCurrent);
    TestingDashboard.getInstance().updateNumber(m_drive, "BackRightMotorCurrent", backRightMotorCurrent);
    TestingDashboard.getInstance().updateNumber(m_drive, "FrontLeftMotorCurrent", frontLeftMotorCurrent);
    TestingDashboard.getInstance().updateNumber(m_drive, "FrontRightMotorCurrent", frontRightMotorCurrent);

    // Trim current buffers until they contain the correct number of entries.
    // Old entries are removed first.
    while (m_left_motor_current_values.size() > m_max_num_current_values) {
      m_left_motor_current_values.remove(0);
    }
    while (m_right_motor_current_values.size() > m_max_num_current_values) {
      m_right_motor_current_values.remove(0);
    }
    TestingDashboard.getInstance().updateNumber(m_drive, "FrontLeftMotorCurrentAverage", getTotalAverageLeftMotorCurrent());
    TestingDashboard.getInstance().updateNumber(m_drive, "FrontRightMotorCurrentAverage", getTotalAverageRightMotorCurrent());
  }

  public static double arrayListAverage(ArrayList<Double> arrayList) {
    double sum = 0;
    for (int i = 0; i < arrayList.size(); i++) {
      sum += arrayList.get(i);
    }
    return sum / arrayList.size();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.DRIVE_PERIODIC_ENABLE) {
      // This method will be called once per scheduler run
      m_accelHelper.captureTimeData();
      m_accelHelper.captureAccelerometerData();
      //accelIntCount += 1;

      if (m_accelHelper.getAccelIntCount() > 2) {
        m_measureVelocity = true;
      }

      if (m_accelHelper.getAccelIntCount() > 4) {
        m_measureDistance = true;
      }

      if (m_measureVelocity) {
        m_accelHelper.calculateVelocity();
      }

      if (m_measureDistance) {
        m_accelHelper.calculateDistance();
      }

      TestingDashboard.getInstance().updateNumber(m_drive, "BackLeftMotorDistance", m_backLeftEncoder.getPosition());
      TestingDashboard.getInstance().updateNumber(m_drive, "BackRightMotorDistance", m_backRightEncoder.getPosition());
      TestingDashboard.getInstance().updateNumber(m_drive, "FrontLeftMotorDistance", m_frontLeftEncoder.getPosition());
      TestingDashboard.getInstance().updateNumber(m_drive, "FrontRightMotorDistance", m_frontRightEncoder.getPosition());
      TestingDashboard.getInstance().updateNumber(m_drive, "BackLeftMotorSpeed", m_backLeftEncoder.getVelocity());
      TestingDashboard.getInstance().updateNumber(m_drive, "BackRightMotorSpeed", m_backRightEncoder.getVelocity());
      TestingDashboard.getInstance().updateNumber(m_drive, "FrontLeftMotorSpeed", m_frontLeftEncoder.getVelocity());
      TestingDashboard.getInstance().updateNumber(m_drive, "FrontRightMotorSpeed", m_frontRightEncoder.getVelocity());
      TestingDashboard.getInstance().updateNumber(m_drive, "currentTime", m_accelHelper.getCurrentTime());
      TestingDashboard.getInstance().updateNumber(m_drive, "instantAccelMagnitudeInchesPerSecondSquared", m_accelHelper.getAccelerometerMagnitudeInchesPerSecondSquared());
      TestingDashboard.getInstance().updateNumber(m_drive, "instantAccelMagnitudeInchesPerSecondSquared", m_accelHelper.getAccelerometerMagnitudeInchesPerSecondSquared());

      // Publish motor current values
      updateMotorCurrentAverages();

      //Remove once PID Controllers are tuned
      setupPIDControllers();
    }
  }
}
