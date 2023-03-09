// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;


public class ArmToPreset extends CommandBase {
  Arm m_arm;
  private double m_turretAngle, m_shoulderAngle, m_elbowAngle, m_wristAngle;
  private boolean m_usingTurret, m_usingShoulder, m_usingElbow, m_usingWrist;


  /** Creates a new ArmToPreset. */
  public ArmToPreset(double turretAngle, double shoulderAngle, double elbowAngle, double wristAngle, boolean usingTurret, boolean usingShoulder, boolean usingElbow, boolean usingWrist) {
    m_arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());

    m_turretAngle = turretAngle;
    m_shoulderAngle = shoulderAngle;
    m_elbowAngle = elbowAngle;
    m_wristAngle = wristAngle;

    m_usingTurret = usingTurret;
    m_usingShoulder = usingShoulder;
    m_usingElbow = usingElbow;
    m_usingWrist = usingWrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_usingTurret) 
      m_arm.setTurretTargetAngle(m_turretAngle);
    if(m_usingShoulder) 
      m_arm.setShoulderTargetAngle(m_shoulderAngle);
    if(m_usingElbow) 
      m_arm.setElbowTargetAngle(m_elbowAngle);
    if(m_usingWrist)
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
    boolean turretFinished = m_arm.getTurretPID().atSetpoint();
    boolean shoulderFinished = m_arm.getShoulderPID().atSetpoint();
    boolean elbowFinished = m_arm.getElbowPID().atSetpoint();
    boolean wristFinished = m_arm.getWristPID().atSetpoint();
    return (turretFinished && shoulderFinished) && (elbowFinished && wristFinished);
  }
}
