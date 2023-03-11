package frc.robot.commands.Auto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Arm.presets.ArmToPreset;
import frc.robot.commands.Arm.sequences.ArmToHomeState;
import frc.robot.commands.Arm.sequences.HighPostCenterState;
import frc.robot.commands.Arm.sequences.HighPostCenterState;
import frc.robot.commands.Hand.ExpelCone;
import frc.robot.commands.Hand.ExpelConeTimed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.TestingDashboard;

public class ScoreCone extends CommandBase {
  enum State {
    INIT,
    SCHEDULE_EXTEND_ARM,
    EXTEND_ARM,
    SCHEDULE_SCORE,
    SCORE,
    SCHEDULE_RETRACT_ARM,
    RETRACT_ARM,
    DONE
  }

  HighPostCenterState m_highPostCenter;
  ExpelConeTimed m_expelConeTimed;
  ArmToHomeState m_armToHome;


  private boolean m_isFinished;
  private State m_state;
  /** Creates a new ReachForNextBarStatefully. */
  public ScoreCone() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_state = State.INIT;
    m_isFinished = false;

    m_highPostCenter = new HighPostCenterState();
    m_expelConeTimed = new ExpelConeTimed(); 
    m_armToHome = new ArmToHomeState();
  }

  //Register with TestingDashboard
  public static void registerWithTestingDashboard() {
    Arm climber = Arm.getInstance();
    ScoreCone cmd = new ScoreCone();
    TestingDashboard.getInstance().registerCommand(climber, "TestCommands", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = State.INIT;
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state) {
      case INIT:
        m_state = State.SCHEDULE_EXTEND_ARM;
        break;

      case SCHEDULE_EXTEND_ARM:
        m_highPostCenter.schedule();
        m_state = State.EXTEND_ARM;
        break;
      case EXTEND_ARM:
        if (m_highPostCenter.isFinished())
          m_state = State.SCHEDULE_SCORE;
        break;

      case SCHEDULE_SCORE:
        m_expelConeTimed.schedule();
        m_state = State.SCORE;
        break;
      case SCORE:
        if (m_expelConeTimed.isFinished())
          m_state = State.SCHEDULE_RETRACT_ARM;
        break;
      
      case SCHEDULE_RETRACT_ARM:
        m_armToHome.schedule();
        m_state = State.RETRACT_ARM;
        break;
      case RETRACT_ARM:
        if (m_armToHome.isFinished())
          m_state = State.DONE;
        break;
      case DONE:
        m_isFinished = true;
        break;
      default:
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
    return m_isFinished;
  }
}

