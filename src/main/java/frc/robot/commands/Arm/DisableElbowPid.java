// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;

public class DisableElbowPid extends CommandBase {
  /** Creates a new DisableArmPid. */
  public DisableElbowPid() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static void registerWithTestingDashboard() {
    Arm arm = Arm.getInstance();
    DisableElbowPid cmd = new DisableElbowPid();
    TestingDashboard.getInstance().registerCommand(arm, "PidJointControl", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.getInstance().disableElbowPid();
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
    return true;
  }
}
