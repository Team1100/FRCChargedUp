// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;


public class ArmToPresetNoTurret extends CommandBase {
  Arm m_arm;
  private double m_shoulderAngle;
  private double m_elbowAngle;
  private double m_wristAngle;


  /** Creates a new ArmToPreset. */
  public ArmToPresetNoTurret(double shoulderAngle, double elbowAngle, double wristAngle) {
    m_arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());

    m_shoulderAngle = shoulderAngle;
    m_elbowAngle = elbowAngle;
    m_wristAngle = wristAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setShoulderTargetAngle(m_shoulderAngle);
    m_arm.setElbowTargetAngle(m_elbowAngle);
    m_arm.setWristTargetAngle(m_wristAngle);

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
    boolean shoulderFinished = m_arm.getShoulderPID().atSetpoint();
    boolean elbowFinished = m_arm.getElbowPID().atSetpoint();
    boolean wristFinished = m_arm.getWristPID().atSetpoint();
    System.out.println(shoulderFinished + " " + elbowFinished + " " + wristFinished);
    return (shoulderFinished) && (elbowFinished && wristFinished);
  }
}
