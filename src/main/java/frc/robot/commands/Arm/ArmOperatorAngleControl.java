// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.input.XboxController;
import frc.robot.input.XboxController.XboxAxis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;

public class ArmOperatorAngleControl extends CommandBase {
  Arm m_arm;
  XboxController m_xbox;

  /** Creates a new ArmOperatorAngleControl. */
  public ArmOperatorAngleControl() {
    m_arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  public static void registerWithTestingDashboard() {
    Arm arm = Arm.getInstance();
    ArmOperatorAngleControl cmd = new ArmOperatorAngleControl();
    TestingDashboard.getInstance().registerCommand(arm, "Manual", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setTurretTargetAngle(0);
    m_arm.setShoulderTargetAngle(0);
    m_arm.setElbowTargetAngle(0);
    m_arm.setWristTargetAngle(0);
    m_xbox = OI.getInstance().getOperatorXboxController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turret is controlled by left and right bumpers
    double t_angle = m_arm.getTurretTargetAngle();
    double w_angle = m_arm.getWristTargetAngle();

    if (m_xbox.getButtonLeftBumper().getAsBoolean()) {
      t_angle += Constants.A_TURRET_ANGLE_INCREMENT;
    } else if (m_xbox.getButtonRightBumper().getAsBoolean()) {
      t_angle += -Constants.A_TURRET_ANGLE_INCREMENT;
    }

    // Wrist is controlled by DPad left and right
    if(m_xbox.getDPad().getRight().getAsBoolean()) {
      w_angle += Constants.A_WRIST_ANGLE_INCREMENT;
    } else if (m_xbox.getDPad().getLeft().getAsBoolean()) {
      w_angle += -Constants.A_WRIST_ANGLE_INCREMENT;
    }

    // Arm joints are controlled by joysticks
    // Note that the max power here is calculated using multiplication
    // because the max power for each arm segment is a value between
    // 0 and 1. Note that 1 / 0.5 == 2, but 1 * 0.5 == 0.5
    double s_angle = m_xbox.getAxis(XboxAxis.kYLeft)*Constants.SHOULDER_DEGREES_PER_JOY_SPAN;
    double e_angle = m_xbox.getAxis(XboxAxis.kYRight)*Constants.DEGREES_PER_JOY_SPAN;

    m_arm.setTurretTargetAngle(t_angle);
    m_arm.setShoulderTargetAngle(s_angle);
    m_arm.setElbowTargetAngle(e_angle);
    m_arm.setWristTargetAngle(w_angle);
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
