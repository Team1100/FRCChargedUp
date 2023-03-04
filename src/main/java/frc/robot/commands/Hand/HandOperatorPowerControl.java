// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hand;

import frc.robot.input.XboxController;
import frc.robot.input.XboxController.XboxAxis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Hand;
import frc.robot.testingdashboard.TestingDashboard;

public class HandOperatorPowerControl extends CommandBase {
  Hand m_hand;
  XboxController m_xbox;
  /** Creates a new HandOperatorPowerControl. */
  public HandOperatorPowerControl() {
    m_hand = Hand.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Hand.getInstance());
  }

  public static void registerWithTestingDashboard() {
    Hand hand = Hand.getInstance();
    HandOperatorPowerControl cmd = new HandOperatorPowerControl();
    TestingDashboard.getInstance().registerCommand(hand, "Manual", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hand.setHandMotorPower(0);
    m_xbox = OI.getInstance().getOperatorXboxController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double h_power = 0;

    if(m_xbox.getAxis(XboxAxis.kRightTrigger) > 0) {
      h_power = m_xbox.getAxis(XboxAxis.kRightTrigger)*Constants.HAND_MAX_POWER;
    }
    if(m_xbox.getAxis(XboxAxis.kLeftTrigger) > 0) {
      h_power = m_xbox.getAxis(XboxAxis.kLeftTrigger)*-Constants.HAND_MAX_POWER;
    }

    m_hand.setHandMotorPower(h_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hand.setHandMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
