// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.presets.ArmToPreset;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeGrabSequence extends SequentialCommandGroup {

  /**
   * Creates a new SequentialArmToPreset. This will be used to score at various
   * preset positions. It rotates the shoulder back while lifting the elbow to 
   * the desired angle, and then moves the shoulder forwards to it's desired angle
   * @param turretAngle
   * @param shoulderAngle
   * @param elbowAngle
   * @param wristAngle
   */
  public ConeGrabSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    
    //TODO: figure out exactly what operators want and reprogram to match
    
    addCommands(
      new ArmToPreset(0, 0, -101, 205, false, true, true, true)
    );
  }

  public static void registerWithTestingDashboard() {
    Arm arm = Arm.getInstance();
    ConeGrabSequence cmd = new ConeGrabSequence();
    TestingDashboard.getInstance().registerCommand(arm, "Automatic", cmd);
  }
}
