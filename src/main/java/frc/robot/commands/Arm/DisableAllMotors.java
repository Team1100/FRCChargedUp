// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;

public class DisableAllMotors extends CommandBase {
  /** Creates a new EnableArmPid. */
  public DisableAllMotors() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static void registerWithTestingDashboard() {
    Arm arm = Arm.getInstance();
    DisableAllMotors cmd = new DisableAllMotors();
    TestingDashboard.getInstance().registerCommand(arm, "Emergency Control", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Arm.getInstance().disableAllMotors();
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
