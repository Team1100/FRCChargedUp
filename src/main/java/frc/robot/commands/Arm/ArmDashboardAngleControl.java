// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;

public class ArmDashboardAngleControl extends CommandBase {
  Arm m_arm;

  /** Creates a new ArmDashboardAngleControl. */
  public ArmDashboardAngleControl() {
    m_arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  public static void registerWithTestingDashboard() {
    Arm arm = Arm.getInstance();
    ArmDashboardAngleControl cmd = new ArmDashboardAngleControl();
    TestingDashboard.getInstance().registerCommand(arm, "Manual", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t_angle = TestingDashboard.getInstance().getNumber(m_arm, "TargetTurretAngle");
    double s_angle = TestingDashboard.getInstance().getNumber(m_arm, "ShoulderTurretAngle");
    double e_angle = TestingDashboard.getInstance().getNumber(m_arm, "ElbowTurretAngle");
    double w_angle = TestingDashboard.getInstance().getNumber(m_arm, "WristTurretAngle");

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
