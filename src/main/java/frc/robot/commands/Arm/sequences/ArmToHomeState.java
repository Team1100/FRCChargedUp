package frc.robot.commands.Arm.sequences;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Arm.presets.ArmToPreset;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;

public class ArmToHomeState extends CommandBase {
  enum State {
    INIT,
    SCHEDULE_RETRACT_SHOULDER,
    RETRACT_SHOULDER,
    SCHEDULE_RETRACT_ELBOW,
    RETRACT_ELBOW,
    SCHEDULE_ROTATE_TURRET,
    ROTATE_TURRET,
    DONE
  }

  ArmToPreset m_retractShoulder = new ArmToPreset(0, 0, 0, 0, false, true, false, false);
  ArmToPreset m_retractElbow = new ArmToPreset(0, 0, 0, 0, false, true, true, true);
  ArmToPreset m_rotateTurret = new ArmToPreset(0, 0, 0, 0, true, true, true, true);

  private double m_shoulderStartingAngle;

  private boolean m_isFinished;
  private boolean m_isPartiallyFinished;
  private State m_state;
  /** Creates a new ReachForNextBarStatefully. */
  public ArmToHomeState() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_state = State.INIT;
    m_isFinished = false;
    m_shoulderStartingAngle = 0;
  }

  //Register with TestingDashboard
  public static void registerWithTestingDashboard() {
    Arm climber = Arm.getInstance();
    ArmToHomeState cmd = new ArmToHomeState();
    TestingDashboard.getInstance().registerCommand(climber, "TestCommands", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = State.INIT;
    m_isFinished = false;
    m_isPartiallyFinished = false;
    m_shoulderStartingAngle = Arm.getInstance().getShoulderAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state) {
      case INIT:
        m_state = State.SCHEDULE_RETRACT_SHOULDER;
        break;

      case SCHEDULE_RETRACT_SHOULDER:
        m_retractShoulder.schedule();
        m_state = State.RETRACT_SHOULDER;
        break;
      case RETRACT_SHOULDER:
        if (Arm.getInstance().isShoulderHalfFinishedGoingIn(m_shoulderStartingAngle))
          m_state = State.SCHEDULE_RETRACT_ELBOW;
          m_isPartiallyFinished = true;
          m_retractShoulder.end(true);
        break;

      case SCHEDULE_RETRACT_ELBOW:
        m_retractElbow.schedule();
        m_state = State.RETRACT_ELBOW;
        break;
      case RETRACT_ELBOW:
        if (m_retractElbow.isFinished())
          m_state = State.SCHEDULE_ROTATE_TURRET;
        break;
      
      case SCHEDULE_ROTATE_TURRET:
        m_rotateTurret.schedule();
        m_state = State.ROTATE_TURRET;
        break;
      case ROTATE_TURRET:
        if (m_rotateTurret.isFinished())
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

  public boolean isPartiallyFinished() {
    return m_isPartiallyFinished;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}

