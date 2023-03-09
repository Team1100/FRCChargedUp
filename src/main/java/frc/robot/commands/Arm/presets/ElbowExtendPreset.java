// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;


public class ElbowExtendPreset extends CommandBase {
  Arm m_arm;
  private double m_turretAngle;
  private double m_shoulderAngle;
  private double m_elbowAngle;
  private double m_wristAngle;


  /** Creates a new ArmToPreset. */
  public ElbowExtendPreset(double elbowAngle) {
    m_arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
    m_elbowAngle = elbowAngle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setElbowTargetAngle(m_elbowAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean elbowFinished = m_arm.getElbowPID().atSetpoint();
    System.out.println(elbowFinished);
    return elbowFinished;
  }

  public boolean isHalfFinishedGoingOut() {
    //boolean elbowFinished = m_arm.getElbowPID().atSetpoint();
    boolean elbowFinished = Math.abs(m_arm.getElbowAngle()) > Math.abs(m_arm.getElbowTargetAngle() / 2);
    System.out.println(elbowFinished);
    return elbowFinished;
  }

  public boolean isHalfFinishedGoingIn() {
    //boolean elbowFinished = m_arm.getElbowPID().atSetpoint();
    boolean elbowFinished = Math.abs(m_arm.getElbowAngle()) < Math.abs(m_arm.getElbowTargetAngle() * 2);
    System.out.println(elbowFinished);
    return elbowFinished;
  }
}
