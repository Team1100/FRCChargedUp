// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ControllerMode;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Intake;
import frc.robot.testingdashboard.TestingDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Arm.presets.*;
import frc.robot.commands.Arm.sequences.ConeGrabSequence;
import frc.robot.commands.Auto.ScoreCone;
import frc.robot.commands.Auto.ScoreConeAndBalance;
import frc.robot.commands.Auto.ScoreConeAndCube;
import frc.robot.commands.Auto.ScoreConeAndDriveBack;
import frc.robot.commands.Auto.ScoreCube;
import frc.robot.commands.Auto.ScoreCubeAndBalance;
import frc.robot.commands.Auto.ScoreCubeAndDriveBack;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Drive.AutoBalanceCommand;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.Drive.DriveFixedSpeed;
import frc.robot.commands.Drive.SwitchDrivePIDMode;
import frc.robot.commands.Drive.ToggleIdleMode;
import frc.robot.commands.Hand.*;
import frc.robot.commands.Intake.IntakePowerControl;
import frc.robot.commands.Lights.ConeLight;
import frc.robot.commands.Lights.CubeLight;
import frc.robot.commands.VisionAuto.DriveToTarget;
import frc.robot.commands.VisionAuto.TrackTarget;
import frc.robot.commands.VisionAuto.TurnToTarget;
import frc.robot.input.ControllerModes.Mode1;
import frc.robot.input.ControllerModes.Mode2;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Drive m_drive;
  private final Arm m_arm;
  private final Hand m_hand;
  private final Intake m_intake;
  private final ControllerMode m_controllerMode;

  private final Command m_scoreConeAndDriveBack = new ScoreConeAndDriveBack(-175,0,0.6);
  private final Command m_scoreConeAndPark = new ScoreConeAndDriveBack(-45,-32,0.6);
  private final Command m_scoreCubeAndDriveBack = new ScoreCubeAndDriveBack(-175,0,0.6);
  private final Command m_scoreCubeAndPark = new ScoreCubeAndDriveBack(-45,-32,0.6);
  private final Command m_scoreCubeAndBalance = new ScoreCubeAndBalance(0,0,0.6);
  private final Command m_scoreConeAndBalance = new ScoreConeAndBalance(0,0,0.6);
  private final Command m_scoreConeAndCube = new ScoreConeAndCube(0.55, 0.4);

  private final Command m_scoreCone = new ScoreCone();
  private final Command m_scoreCube = new ScoreCube();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drive = Drive.getInstance();
    m_drive.setDefaultCommand(new ArcadeDrive());

    m_arm = Arm.getInstance();
    m_arm.setDefaultCommand(new ArmOperatorRelativeAngleControl());

    m_hand = Hand.getInstance();
    m_hand.setDefaultCommand(new HandOperatorPowerControl());

    m_intake = Intake.getInstance();
    m_intake.setDefaultCommand(new IntakePowerControl());

    m_controllerMode = ControllerMode.getInstance();
    m_controllerMode.setDefaultCommand(OI.getInstance().getMode1());

    // Auto Routines: 
    m_chooser.setDefaultOption("Cone and Drive Back", m_scoreConeAndDriveBack);
    m_chooser.addOption("Cone And Park", m_scoreConeAndPark);
    m_chooser.addOption("Cone", m_scoreCone);
    m_chooser.addOption("Cube And Drive Back", m_scoreCubeAndDriveBack);
    m_chooser.addOption("Cube And Park", m_scoreCubeAndPark);
    m_chooser.addOption("Cube", m_scoreCube);
    m_chooser.addOption("Cube And Balance", m_scoreCubeAndBalance);
    m_chooser.addOption("Cone and Balance", m_scoreConeAndBalance);
    m_chooser.addOption("Cone and Cube", m_scoreConeAndCube);
    // Configure the trigger bindings
    configureBindings();

    // Register commands with TestingDashboard commands

    // Auto

    // Drive
    ArcadeDrive.registerWithTestingDashboard();
    ToggleIdleMode.registerWithTestingDashboard();
    DriveToTarget.registerWithTestingDashboard();
    // Arm
    ArmOperatorPowerControl.registerWithTestingDashboard();
    ArmOperatorAngleControl.registerWithTestingDashboard();
    ArmOperatorRelativeAngleControl.registerWithTestingDashboard();
    ArmDashboardAngleControl.registerWithTestingDashboard();
    EnableArmPid.registerWithTestingDashboard();
    DisableArmPid.registerWithTestingDashboard();
    EnableWristPid.registerWithTestingDashboard();
    DisableWristPid.registerWithTestingDashboard();
    EnableElbowPid.registerWithTestingDashboard();
    DisableElbowPid.registerWithTestingDashboard();
    EnableShoulderPid.registerWithTestingDashboard();
    DisableShoulderPid.registerWithTestingDashboard();
    EnableTurretPid.registerWithTestingDashboard();
    DisableTurretPid.registerWithTestingDashboard();
    ConeGrabSequence.registerWithTestingDashboard();
    ZeroArmEncoders.registerWithTestingDashboard();
    ArmToHomePosition.registerWithTestingDashboard();
    CancelArmMovement.registerWithTestingDashboard();
    SwitchDrivePIDMode.registerWithTestingDashboard();
    DriveFixedSpeed.registerWithTestingDashboard();
    AutoBalanceCommand.registerWithTestingDashboard();

    // Hand
    // SpinIntake.registerWithTestingDashboard();
    ExpelCone.registerWithTestingDashboard();
    ExpelCube.registerWithTestingDashboard();
    IntakeCone.registerWithTestingDashboard();
    IntakeCube.registerWithTestingDashboard();
    HandOperatorPowerControl.registerWithTestingDashboard();
    SmartIntakeCube.registerWithTestingDashboard();
    // Controller Modes
    Mode1.registerWithTestingDashboard();
    Mode2.registerWithTestingDashboard();

    // Lights
    ConeLight.registerWithTestingDashboard();
    CubeLight.registerWithTestingDashboard();

    // Vision
    TurnToTarget.registerWithTestingDashboard();

    // Create Testing Dashboard
    TestingDashboard.getInstance().createTestingDashboard();
    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    OI.getInstance();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
