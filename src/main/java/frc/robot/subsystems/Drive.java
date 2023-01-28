// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.Drive.ArcadeDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Drive extends SubsystemBase {

  private CANSparkMax m_frontLeft;
  private CANSparkMax m_frontRight;
  private RelativeEncoder m_frontLeftEncoder;
  private RelativeEncoder m_frontRightEncoder;
  private DifferentialDrive drivetrain;

  private boolean m_measureVelocity;
  private boolean m_measureDistance;
  private double accelIntCount = 0;
  private IdleMode m_currentIdleMode;

  private AnalogInput potentiometer;

  private SlewRateLimiter fwdRateLimiter;
  private SlewRateLimiter rotRateLimiter;
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
  public static final double GEAR_RATIO = 8.68; //number of times the motor rotates to rotate wheel once
  public static final double CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE / GEAR_RATIO; //conversion factor * circumference = distance
  public final static double DISTANCE = CONVERSION_FACTOR * WHEEL_CIRCUMFERENCE;
  public final static double INITIAL_SPEED = 0.3;
  
  private static Drive m_drive;

  /** Creates a new Drive. */
  public Drive() {

    //Add Potentiometer
    potentiometer = new AnalogInput(RobotMap.D_SPEED_POTENTIOMETER);

    m_frontLeft = new CANSparkMax(RobotMap.D_FRONT_LEFT, MotorType.kBrushless);
    m_frontRight = new CANSparkMax(RobotMap.D_FRONT_RIGHT, MotorType.kBrushless);

    m_frontLeftEncoder = m_frontLeft.getEncoder();
    m_frontRightEncoder = m_frontRight.getEncoder();

    m_frontLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults(); 

    m_frontLeft.setInverted(false);
    m_frontRight.setInverted(true);

    drivetrain = new DifferentialDrive(m_frontLeft, m_frontRight);

    setIdleMode(IdleMode.kCoast);
    m_currentIdleMode = IdleMode.kCoast;
    setEncoderConversionFactor(CONVERSION_FACTOR);

    fwdRateLimiter = new SlewRateLimiter(fwdRateLimit);
    rotRateLimiter = new SlewRateLimiter(rotRateLimit);
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
    
    if(m_frontLeft.setIdleMode(mode) != REVLibError.kOk){
      System.out.println("Could not set idle mode on front left motor");
      //System.exit(1);
    }
  
    if(m_frontRight.setIdleMode(mode) != REVLibError.kOk){
      System.out.println("Could not set idle mode on front right motor");
      //System.exit(1);
    }
  }

  public void setEncoderConversionFactor(double conversionFactor) {

    if(m_frontLeftEncoder.setPositionConversionFactor(conversionFactor) != REVLibError.kOk){ 
      System.out.println("Could not set position conversion factor on front left encoder");
    }
  
    if(m_frontRightEncoder.setPositionConversionFactor(conversionFactor) != REVLibError.kOk){
      System.out.println("Could not set position conversion factor on front right encoder");
    } 
  }

  public static Drive getInstance() {
    if (m_drive == null) {
      m_drive = new Drive();
    }
    return m_drive;
  }

  public void switchIdleMode() {
    if (m_currentIdleMode == IdleMode.kCoast) {
      setIdleMode(IdleMode.kBrake);
      m_currentIdleMode = IdleMode.kBrake;
    } else if (m_currentIdleMode == IdleMode.kBrake) {
      setIdleMode(IdleMode.kCoast);
      m_currentIdleMode = IdleMode.kCoast;
    }
  }

  //Drive Methods:

  public double getPercentPower() {
    double percent = potentiometer.getVoltage() / 5;
    return percent;
  }

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

  public void arcadeDrive(double fwd, double rot, boolean sqInputs) {
    drivetrain.arcadeDrive(fwd, rot, sqInputs);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_rightSpeed = rightSpeed;
    m_leftSpeed = leftSpeed;
    drivetrain.tankDrive(leftSpeed, rightSpeed);
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
      new ArcadeDrive();
      // This is just to be used for debugging
      double fwdLimit = SmartDashboard.getNumber("FWD Accel Limit", 3);
      double rotLimit = SmartDashboard.getNumber("ROT Accel Limit", 3);

      if (fwdLimit != fwdRateLimit) {
        fwdRateLimiter = new SlewRateLimiter(fwdLimit);
        fwdRateLimit = fwdLimit;
      }
      if (rotLimit != rotRateLimit) {
        rotRateLimiter = new SlewRateLimiter(rotLimit);
        rotRateLimit = rotLimit;
      }
    }
  }
}
