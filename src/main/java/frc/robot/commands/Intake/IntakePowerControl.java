// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.input.XboxController;
import frc.robot.input.XboxController.XboxAxis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Intake;
import frc.robot.testingdashboard.TestingDashboard;

public class IntakePowerControl extends CommandBase {
  Intake m_intake;
  XboxController m_xbox;
  /** Creates a new IntakePowerControl. */
  public IntakePowerControl() {
    m_intake = Intake.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake.getInstance());
  }

  public static void registerWithTestingDashboard() {
    Intake intake = Intake.getInstance();
    IntakePowerControl cmd = new IntakePowerControl();
    TestingDashboard.getInstance().registerCommand(intake, "Manual", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeMotorPower(0);
    m_intake.setIntakeWinchPower(0);
    m_xbox = OI.getInstance().getDriverXboxController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_power = 0;
    double w_power = -Constants.WINCH_MAX_POWER;;
    m_intake.setWinchBrake();
    if(m_xbox.getAxis(XboxAxis.kRightTrigger) > 0) {
      m_power = m_xbox.getAxis(XboxAxis.kRightTrigger)*-Constants.INTAKE_MAX_POWER;
      w_power = 0;
      m_intake.setWinchCoast();
    }
    if(m_xbox.getAxis(XboxAxis.kLeftTrigger) > 0) {
      m_power = m_xbox.getAxis(XboxAxis.kLeftTrigger)*Constants.INTAKE_MAX_POWER;
      w_power = 0;
      m_intake.setWinchCoast();
    }

    if (m_intake.inAuto) {
      w_power = 0;
      m_intake.setWinchCoast();
    }

    
    m_intake.setIntakeMotorPower(m_power);
    m_intake.setIntakeWinchPower(w_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeMotorPower(0);
    m_intake.setIntakeWinchPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
