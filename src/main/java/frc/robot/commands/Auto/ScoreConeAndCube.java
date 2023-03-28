package frc.robot.commands.Auto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Arm.ReversedSequences.ReversedFloorGrabSequenceCube;
import frc.robot.commands.Arm.sequences.ArmToHomeState;
import frc.robot.commands.Arm.sequences.HighPostCenterState;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.Hand.ExpelConeTimed;
import frc.robot.commands.Hand.ExpelCubeTimed;
import frc.robot.commands.Hand.SmartIntakeCube;
import frc.robot.commands.VisionAuto.DriveToTarget;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.TestingDashboard;

public class ScoreConeAndCube extends CommandBase {
  enum State {
    // Part 1: score the cone and drive back
    INIT,
    SCHEDULE_EXTEND_ARM,
    EXTEND_ARM,
    SCHEDULE_SCORE,
    SCORE,
    SCHEDULE_RETRACT_ARM_AND_DRIVE_BACK,
    RETRACT_ARM_AND_DRIVE_BACK,
    SCHEDULE_DRIVE_TO_CUBE,
    DRIVE_TO_CUBE,
    // Part 2: pickup cube off the floor
    SCHEDULE_PICK_UP_CUBE,
    PICK_UP_CUBE,

    // Part 3: drive back, turn, score the cube
    SCHEDULE_DRIVE_BACK,
    DRIVE_BACK,
    //Add a turn command
    SCHEDULE_DRIVE_TO_TAG,
    DRIVE_TO_TAG,
    SCHEDULE_EXTEND_ARM_2,
    EXTEND_ARM_2,
    SCHEDULE_SCORE_2,
    SCORE_2,
    SCHEDULE_RETRACT_ARM_2,
    RETRACT_ARM_2,
    DONE
  }

  HighPostCenterState m_highPostCenter;
  ExpelConeTimed m_expelConeTimed;
  ArmToHomeState m_armToHome;
  DriveDistance m_driveBack;
  ReversedFloorGrabSequenceCube m_floorGrabSequence;
  TimedFloorPickup m_timedFloorPickup;
  SmartIntakeCube m_smartIntakeCube;
  DriveDistance m_driveBack2;
  //Add the turn command
  ExpelCubeTimed m_expelCubeTimed;

  DriveToTarget m_driveToCube;
  DriveToTarget m_driveToTag;


  private boolean m_isFinished;
  private State m_state;
  /** Creates a new ReachForNextBarStatefully. */
  public ScoreConeAndCube(double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_state = State.INIT;
    m_isFinished = false;

    // Part 1 of the sequence
    m_highPostCenter = new HighPostCenterState();
    m_expelConeTimed = new ExpelConeTimed(); 
    m_armToHome = new ArmToHomeState();
    m_driveBack = new DriveDistance(-36, power, power, 0, true);
    m_driveToCube = new DriveToTarget(-264, power, power, 0, true);
    // Part 2 of the sequence
    m_floorGrabSequence = new ReversedFloorGrabSequenceCube();
    m_smartIntakeCube = new SmartIntakeCube();
    // Part 3 of the sequence
    m_driveBack2 = new DriveDistance(36, power, power, 0, true);
    m_driveToTag = new DriveToTarget(264, power, power, 0, true);

    m_expelCubeTimed = new ExpelCubeTimed();

  }

  //Register with TestingDashboard
  public static void registerWithTestingDashboard() {
    Arm climber = Arm.getInstance();
    ScoreConeAndDriveBack cmd = new ScoreConeAndDriveBack(0,0,0);
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

      // Part 1 of the sequence
      case INIT:
        m_state = State.SCHEDULE_EXTEND_ARM;
        break;

      case SCHEDULE_EXTEND_ARM:
        m_highPostCenter.schedule();
        m_state = State.EXTEND_ARM;
        break;
      case EXTEND_ARM:
        if (m_highPostCenter.isFinished()) {
          m_state = State.SCHEDULE_SCORE;
        }
        break;

      case SCHEDULE_SCORE:
        m_expelConeTimed.schedule();
        m_state = State.SCORE;
        break;
      case SCORE:
        if (m_expelConeTimed.isFinished()) {
          m_state = State.SCHEDULE_RETRACT_ARM_AND_DRIVE_BACK;
        }
        break;
      
      case SCHEDULE_RETRACT_ARM_AND_DRIVE_BACK:
        m_armToHome.schedule();
        m_driveBack.schedule();
        m_state = State.RETRACT_ARM_AND_DRIVE_BACK;
        break;
      case RETRACT_ARM_AND_DRIVE_BACK:
        if (m_driveBack.isFinished()) {
          m_state = State.SCHEDULE_DRIVE_TO_CUBE;
        }
        break;

      case SCHEDULE_DRIVE_TO_CUBE:
        Vision.getInstance().setDetectionMode(Vision.DETECTING_COLOR);
        m_driveToCube.schedule();
        m_state = State.DRIVE_TO_CUBE;
        break;
      case DRIVE_TO_CUBE:
        if (m_driveToCube.isPartiallyFinished(.75)) {
          m_state = State.SCHEDULE_PICK_UP_CUBE;
        }
        break;

      // Part 2 of the sequence, states to pick up cube
      case SCHEDULE_PICK_UP_CUBE:
        m_floorGrabSequence.schedule();
        m_smartIntakeCube.schedule();
        m_driveToCube.setPower(0.2, 0.2);
        m_state = State.PICK_UP_CUBE;
        break;
      case PICK_UP_CUBE:
        if (m_smartIntakeCube.isFinished() || m_driveToCube.isFinished()) {
          m_state = State.SCHEDULE_DRIVE_BACK;
        }
        break;

      // Part 3 of the sequence, states to drive back and score
      case SCHEDULE_DRIVE_BACK:
        Vision.getInstance().setDetectionMode(Vision.DETECTING_APRILTAG);
        m_driveBack2.schedule();
        m_armToHome.schedule();
        m_state = State.DRIVE_BACK;
      case DRIVE_BACK:
        if (m_driveBack2.isFinished()) {
          m_state = State.SCHEDULE_DRIVE_TO_TAG;
        }
        break;
      case SCHEDULE_DRIVE_TO_TAG:
          m_driveToTag.schedule();
          m_state = State.DRIVE_TO_TAG;
        
        break;
      case DRIVE_TO_TAG:
        if (m_driveToTag.isPartiallyFinished(.75)) {
          m_state = State.SCHEDULE_EXTEND_ARM_2;
        }
        break;
      
      case SCHEDULE_EXTEND_ARM_2:
        m_highPostCenter.schedule();
        m_state = State.EXTEND_ARM_2;
        break;
      case EXTEND_ARM_2:
        if (m_highPostCenter.isFinished() && m_driveToTag.isFinished()) {
          m_state = State.SCHEDULE_SCORE_2;
        }
        break;

      case SCHEDULE_SCORE_2:
        m_expelCubeTimed.schedule();
        m_state = State.SCORE_2;
        break;
      case SCORE_2:
        if (m_expelCubeTimed.isFinished()) {
          m_state = State.SCHEDULE_RETRACT_ARM_2;
        }
        break;
      
      case SCHEDULE_RETRACT_ARM_2:
        m_armToHome.schedule();
        m_state = State.RETRACT_ARM_2;
        break;
      case RETRACT_ARM_2:
        if (m_armToHome.isFinished()) {
          m_state = State.DONE;
        }
        break;
      
      // Done with the sequence

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

