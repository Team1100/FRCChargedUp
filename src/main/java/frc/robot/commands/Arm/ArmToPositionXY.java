// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmToPositionXY extends CommandBase {

  Arm m_arm;

  // Creates a new ArmToPositionXY:
  public ArmToPositionXY() {
    m_arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  public static void registerWithTestingDashboard() {
    Arm arm = Arm.getInstance();
    ArmToPositionXY cmd = new ArmToPositionXY();
    TestingDashboard.getInstance().registerCommand(arm, "Automatic", cmd);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Sets the target angles to the values on TD
    m_arm.setShoulderTargetAngle(TestingDashboard.getInstance().getNumber(m_arm, "TargetShoulderAngle"));
    m_arm.setElbowTargetAngle(TestingDashboard.getInstance().getNumber(m_arm, "TargetElbowAngle"));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
