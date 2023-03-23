// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.TestingDashboard;

public class DriveFixedSpeed extends CommandBase {

  private Drive m_Drive;
  private double m_targetSpeed;
  /** Creates a new DriveFixedSpeed. */
  public DriveFixedSpeed() {
    m_Drive = Drive.getInstance();
    addRequirements(m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double targetRpm = TestingDashboard.getInstance().getNumber(m_Drive, "DriveSpeedRPM");
    m_targetSpeed = targetRpm/Constants.DRIVE_MAX_MOTOR_RPM;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetRpm = TestingDashboard.getInstance().getNumber(m_Drive, "DriveSpeedRPM");
    m_targetSpeed = targetRpm/Constants.DRIVE_MAX_MOTOR_RPM;
    m_Drive.tankDrive(m_targetSpeed, m_targetSpeed);
  }

  public static void registerWithTestingDashboard()
  {
    DriveFixedSpeed cmd  = new DriveFixedSpeed();
    TestingDashboard.getInstance().registerCommand(Drive.getInstance(), "Basic", cmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
