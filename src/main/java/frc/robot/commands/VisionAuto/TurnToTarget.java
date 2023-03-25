// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.VisionAuto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.TestingDashboard;

public class TurnToTarget extends CommandBase {
  static Drive m_drive;
  static Vision m_vision;
  private static final double TARGET_TOLERANCE = 50;
  private double m_speed;
  
  /** Creates a new DriveToTarget. */
  public TurnToTarget(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = Drive.getInstance();
    m_vision = Vision.getInstance();
    addRequirements(m_vision);
    
    
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double direction = -(int) Math.abs(m_vision.getTargetOffset())/m_vision.getTargetOffset();
    m_drive.tankDrive(m_speed  * direction, m_speed * -direction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double direction = (int) Math.abs(m_vision.getTargetOffset())/m_vision.getTargetOffset();
    m_drive.tankDrive(m_speed * direction, m_speed * -direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double offset = m_vision.getTargetOffset();
    return offset < TARGET_TOLERANCE && offset > -TARGET_TOLERANCE;
  }

    //Register with TestingDashboard
    public static void registerWithTestingDashboard() {
      Vision vision = Vision.getInstance();
      TurnToTarget cmd = new TurnToTarget(.3);
      TestingDashboard.getInstance().registerCommand(vision, "TargetTracking", cmd);
    }
}
