// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionAuto;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.testingdashboard.TestingDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DriveToTarget extends CommandBase {
  static Drive m_drive;
  boolean m_parameterized;
  double m_distance;
  double m_leftPower;
  double m_rightPower;
  boolean m_finished;
  double m_brakeDelay;

  private Timer m_timer;
  double ENCODER_INITIAL_POSITION = 0;
  private final double NUDGE_FACTOR = 0.027;
  private static final double TARGET_TOLERANCE = 10;
  double defaultLeftPower;
  double defaultRightPower;

  static Vision m_vision;


  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;

  /** Creates a new DriveDistance. */
  public DriveToTarget(double distance, double lPower, double rPower, double brakeDelay, boolean parameterized) {
    m_drive = Drive.getInstance();
    m_vision = Vision.getInstance();
    defaultLeftPower = lPower;
    defaultRightPower = rPower;
    m_leftPower = lPower;
    m_rightPower = rPower;
    m_parameterized = parameterized;
    m_distance = 1.636 * distance;
    m_timer = new Timer();
    m_brakeDelay = brakeDelay;
    m_finished = false;

    m_leftEncoder = m_drive.getLeftEncoder();
    m_rightEncoder = m_drive.getRightEncoder();
    m_leftEncoder.setPosition(ENCODER_INITIAL_POSITION);
    m_rightEncoder.setPosition(ENCODER_INITIAL_POSITION);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  public static void registerWithTestingDashboard() {
    Drive drive = Drive.getInstance();
    DriveToTarget cmd = new DriveToTarget(12.0, Constants.D_AUTO_DRIVE_SPEED, Constants.D_AUTO_DRIVE_SPEED, Constants.D_BRAKE_DELAY, false);
    TestingDashboard.getInstance().registerCommand(drive, "Basic", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drive.setIdleMode(IdleMode.kBrake);
    m_drive.tankDrive(0, 0);
    m_leftEncoder.setPosition(ENCODER_INITIAL_POSITION);
    m_rightEncoder.setPosition(ENCODER_INITIAL_POSITION);
    m_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_nudgeFactor = NUDGE_FACTOR;
    if (!m_parameterized) {
      m_distance = TestingDashboard.getInstance().getNumber(m_drive, "DistanceToTravelInInches");
      m_nudgeFactor = TestingDashboard.getInstance().getNumber(m_drive, "NudgeFactor");
      m_brakeDelay = Constants.D_BRAKE_DELAY;
    }
    m_leftPower = defaultLeftPower;
    m_rightPower = defaultRightPower;
    double offset = m_vision.getTargetOffset();
    if (!(offset < TARGET_TOLERANCE && offset > -TARGET_TOLERANCE)) {
      double direction = (int) Math.abs(m_vision.getTargetOffset())/m_vision.getTargetOffset();
      m_leftPower += m_nudgeFactor * direction;
      m_rightPower -= m_nudgeFactor * direction;
    }
      
    if (m_distance >= 0) {
      m_drive.tankDrive(m_leftPower, m_rightPower);
    } else {
      m_drive.tankDrive(-m_leftPower, -m_rightPower);
    }
    System.out.println(m_leftEncoder.getPosition() + "   " + m_rightEncoder.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_finished) {
      if (m_distance >= 0) {
        if (m_leftEncoder.getPosition() >= m_distance || m_rightEncoder.getPosition() >= m_distance) {
          m_finished = true;
        }
      } else if (m_distance < 0) {
        if (m_leftEncoder.getPosition() <= m_distance || m_rightEncoder.getPosition() <= m_distance) {
          m_finished = true;
        }
      }
    }
    if (m_finished) {
      m_drive.tankDrive(0, 0);
    }
    // if the timer has elapsed the delay and the command is finished, the command can end
    return m_finished;
  }
}
