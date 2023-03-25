// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionAuto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.TestingDashboard;

public class TrackTarget extends CommandBase {
  // Thomas, was, in, fact, here;
  Arm m_arm;
  Vision m_vision;

  /** Creates a new TrackTarget. */
  public TrackTarget() {
    m_arm = Arm.getInstance();
    m_vision = Vision.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.enableLimeLight(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Queries Vision subsystem for the vision values and acts upon the returned values:
    if (m_vision.isAprilTagTargetFound()) {
      // Using isTargetFound to prevent turret from moving unnecessarily
      m_arm.setTurretTargetAngle(m_arm.getTurretAngle() + m_vision.getTargetYaw());
      // TODO: add support for X/Y positioning as well
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vision.enableLimeLight(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
