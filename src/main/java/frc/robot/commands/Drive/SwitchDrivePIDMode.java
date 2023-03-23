// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.TestingDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwitchDrivePIDMode extends InstantCommand {

  private Drive m_drive;

  public SwitchDrivePIDMode() {
    m_drive = Drive.getInstance();
  }

  public static void registerWithTestingDashboard()
  {
    SwitchDrivePIDMode cmd  = new SwitchDrivePIDMode();
    TestingDashboard.getInstance().registerCommand(Drive.getInstance(), "Basic", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute()
  {
    m_drive.togglePIDDriveMode();
  }
}
