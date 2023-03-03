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

  public static void registerWithTestingDashboard() {
    Arm arm = Arm.getInstance();
    Vision vision = Vision.getInstance();
    TrackTarget cmd = new TrackTarget();
    TestingDashboard.getInstance().registerCommand(vision, "Automatic", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Queries Vision subsystem for the vision values and acts upon the returned values:

    while(m_vision.isTargetFound()) {             // Using isTargetFound to prevent turret from moving unnecessarily
    double offset = m_vision.getTargetOffset();
    
    // Example of how this system should work: 

    if(offset > 10) {
      m_arm.setTurretTargetAngle(m_arm.getTurretAngle() - Constants.A_TURRET_ANGLE_INCREMENT);
    } else if(offset < -10) {
      m_arm.setTurretTargetAngle(m_arm.getTurretAngle() + Constants.A_TURRET_ANGLE_INCREMENT);
    }

    // NOT FINISHED.  DO NOT USE YET!!!!!!!!  DON'T BREAK THE ROBOT!!!
    m_arm.setShoulderTargetAngle(m_arm.getShoulderAngle() + Constants.A_SHOULDER_ANGLE_INCREMENT);
    m_arm.setElbowTargetAngle(m_arm.getElbowAngle() + Constants.A_ELBOW_ANGLE_INCREMENT);
  }
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
