// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;

public class ZeroArmEncoders extends CommandBase {
  Arm m_arm;
  
  /** Creates a new ZeroArmEncoders. */
  public ZeroArmEncoders() {
    m_arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static void registerWithTestingDashboard() {
    Arm arm = Arm.getInstance();
    ZeroArmEncoders cmd = new ZeroArmEncoders();
    TestingDashboard.getInstance().registerCommand(arm, "Manual", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.zeroEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
