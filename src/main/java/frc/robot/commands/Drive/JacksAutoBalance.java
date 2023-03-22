// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.TestingDashboard;

public class JacksAutoBalance extends CommandBase {
  private Drive m_drive;
  private boolean m_finished;
  private double m_current;
  private final double HALF_CURRENT = 15;
  private final double MAX_CURRENT = 50;
  private state m_state;

  enum state{
    AWAITING,
    OVER_BACKWARDS,
    OVER_FORWARDS,
    DONE
  }

  /** Creates a new JacksAutoBalance. */
  public JacksAutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  public static void registerWithTestingDashboard() {
    JacksAutoBalance cmd = new JacksAutoBalance();
    Drive drive = Drive.getInstance();
    TestingDashboard.getInstance().registerCommand(drive, "balance", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive = Drive.getInstance();
    m_finished = false;
    m_state = state.AWAITING;
    m_current = m_drive.getInstantTotalMotorCurrent();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_current = m_drive.getInstantTotalMotorCurrent();

    switch(m_state) {

      case AWAITING:
        m_drive.tankDrive(-Constants.D_AUTO_DRIVE_SPEED, -Constants.D_AUTO_DRIVE_SPEED);
        if(m_current > HALF_CURRENT)
          m_state = state.OVER_BACKWARDS;
        break;

      case OVER_BACKWARDS:
        double multiplier = 0.8;
        double speed = Math.abs(((m_current / MAX_CURRENT) - (HALF_CURRENT / MAX_CURRENT)) * multiplier);
        if(speed > 0.5) {
          speed = 0.5;
        }
        m_drive.tankDrive(speed, speed);
        if(m_current < HALF_CURRENT)
          m_state = state.OVER_FORWARDS;
        break;

      case OVER_FORWARDS:
        double multiplier2 = 0.8;
        double speed2 = -Math.abs(((m_current / MAX_CURRENT) - (HALF_CURRENT / MAX_CURRENT)) * multiplier2);
        if(speed2 < -0.5) {
          speed2 = -0.5;
        }
        m_drive.tankDrive(speed2, speed2);
        if(m_current < HALF_CURRENT)
        m_state = state.DONE;
        break;

      case DONE:
        m_drive.tankDrive(0, 0);
        m_drive.setIdleMode(IdleMode.kBrake);
        m_finished = true;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
