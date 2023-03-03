// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hand;
import frc.robot.testingdashboard.TestingDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Arm.presets.*;
import frc.robot.commands.Arm.sequences.ArmToPresetBackwardSequence;
import frc.robot.commands.Arm.sequences.ArmToPresetForwardSequence;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Hand.*;

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drive = Drive.getInstance();
    m_drive.setDefaultCommand(new ArcadeDrive());

    m_arm = Arm.getInstance();
    m_arm.setDefaultCommand(new ArmOperatorRelativeAngleControl());

    m_hand = Hand.getInstance();
    m_hand.setDefaultCommand(new HandOperatorPowerControl());

    // Configure the trigger bindings
    configureBindings();

    // Register commands with TestingDashboard commands

    // Drive
    ArcadeDrive.registerWithTestingDashboard();

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
    EnableAllMotors.registerWithTestingDashboard();
    DisableAllMotors.registerWithTestingDashboard();
    ArmToPresetForwardSequence.registerWithTestingDashboard();
    ArmToPresetBackwardSequence.registerWithTestingDashboard();
    ArmToPreset1.registerWithTestingDashboard();
    ArmToPreset2.registerWithTestingDashboard();
    ArmToPreset3.registerWithTestingDashboard();
    ArmToPreset4.registerWithTestingDashboard();
    ArmToPreset5.registerWithTestingDashboard();
    ArmToPreset6.registerWithTestingDashboard();
    ArmToPreset7.registerWithTestingDashboard();
    ArmToPreset8.registerWithTestingDashboard();
    ZeroArmEncoders.registerWithTestingDashboard();

    // Hand
    SpinIntake.registerWithTestingDashboard();
    ExpelCone.registerWithTestingDashboard();
    ExpelCube.registerWithTestingDashboard();
    IntakeCone.registerWithTestingDashboard();
    IntakeCube.registerWithTestingDashboard();
    HandOperatorPowerControl.registerWithTestingDashboard();

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
