// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.TestingDashboard;
import frc.robot.commands.Drive.DriveDistance;

public class TurnAngle extends DriveDistance {

  /** Creates a new TurnAngle. */
  public TurnAngle(double leftSpeed, double rightSpeed, double angle) {
    super(angle / 2, leftSpeed, -rightSpeed, 0, true);
  }

  public static void registerWithTestingDashboard() {
    Drive drive = Drive.getInstance();
    TurnAngle cmd = new TurnAngle(12, 0.3, 0.311);
    TestingDashboard.getInstance().registerCommand(drive, "Turning", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
