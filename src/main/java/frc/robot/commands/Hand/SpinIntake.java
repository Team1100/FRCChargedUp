// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hand;
import frc.robot.testingdashboard.TestingDashboard;

public class SpinIntake extends CommandBase {

  protected Hand m_hand;
  private boolean m_finished;
  private double m_speed = Constants.DEFAULT_INTAKE_CONE_SPEED;
  
  /** Creates a new IntakeBall. */
  public SpinIntake(double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hand = Hand.getInstance();
    addRequirements(m_hand);
    m_finished = false;
    m_speed = s;
  }

  //Register with TestingDashboard
  public static void registerWithTestingDashboard() {
    Hand hand = Hand.getInstance();
    SpinIntake cmd = new SpinIntake(Constants.DEFAULT_INTAKE_CONE_SPEED);
    TestingDashboard.getInstance().registerCommand(hand, "Basic", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hand.spinHandMotor(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hand.spinHandMotor(0);
    m_finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
