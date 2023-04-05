// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.helpers.VelocityDriveSparkMax.DriveMode;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.TestingDashboard;

public class AutoBalanceCommand extends CommandBase {

  Drive m_drive;
  AutoBalance m_autoBalance;
  int m_direction;
  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand(int direction) {
    m_drive = Drive.getInstance();
    addRequirements(m_drive);
    m_autoBalance = new AutoBalance(direction);
    m_direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autoBalance.clearFinished();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0;
    if (m_direction == Constants.BALANCING_BACKWARD) {
      speed = m_autoBalance.backwardAutoBalanceRoutine();
    } else if (m_direction == Constants.BALANCING_FORWARD) {
      speed = m_autoBalance.forwardAutoBalanceRoutine();
    }
    
    m_drive.setDriveMode(DriveMode.kPower);
    m_drive.tankDrive(speed, speed);
  }

  public static void registerWithTestingDashboard(){
    AutoBalanceCommand cmd = new AutoBalanceCommand(Constants.BALANCING_BACKWARD);
    TestingDashboard.getInstance().registerCommand(Drive.getInstance(), "Basic", cmd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_autoBalance.clearFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_autoBalance.isFinished();
  }
}
