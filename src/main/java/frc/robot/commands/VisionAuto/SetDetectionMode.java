// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionAuto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.TestingDashboard;

public class SetDetectionMode extends CommandBase {
  /** Creates a new SetDetectionMode. */
  public SetDetectionMode() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Vision.getInstance().setDetectionMode(Vision.DETECTING_APRILTAG);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

      //Register with TestingDashboard
      public static void registerWithTestingDashboard() {
        Vision vision = Vision.getInstance();
        SetDetectionMode cmd = new SetDetectionMode();
        TestingDashboard.getInstance().registerCommand(vision, "TargetTracking", cmd);
      }
}
