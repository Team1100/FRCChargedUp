// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.input.ControllerModes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.ControllerMode;
import frc.robot.testingdashboard.TestingDashboard;

public class Mode3 extends CommandBase {
  /** Creates a new Mode3. */
  private boolean m_isFinished;

  public Mode3() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ControllerMode.getInstance());
  }

  public static void registerWithTestingDashboard() {
    ControllerMode controllerMode = ControllerMode.getInstance();
    Mode3 cmd = new Mode3();
    TestingDashboard.getInstance().registerCommand(controllerMode, "Modes", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished = false;  

    TestingDashboard.getInstance().updateString(ControllerMode.getInstance(), "CurrentMode", "Mode3");

    OI.getInstance().Mode3();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(OI.getInstance().getOperatorXboxController().getDPad().getRight().getAsBoolean()) {
      OI.getInstance().getMode1().schedule();
      m_isFinished = true;
    } else if (OI.getInstance().getOperatorXboxController().getDPad().getLeft().getAsBoolean()) {
      OI.getInstance().getMode2().schedule();
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
