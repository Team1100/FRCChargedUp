// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AutoBalanceCommand extends CommandBase {

  Drive m_drive;
  AutoBalance m_autoBalance;
  /** Creates a new AutoBalanceCommand. */
  public AutoBalanceCommand() {
    m_drive = Drive.getInstance();
    addRequirements(m_drive);
    m_autoBalance = new AutoBalance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_autoBalance.autoBalanceRoutine();
    m_drive.tankDrive(speed, speed);
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
