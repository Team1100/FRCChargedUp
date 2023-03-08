// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.presets.ArmToPreset;
import frc.robot.commands.Arm.presets.ArmToPreset1;
import frc.robot.commands.Arm.presets.ArmToPresetNoTurret;
import frc.robot.commands.Arm.presets.ElbowExtendPreset;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeHighPostCenterSequence extends SequentialCommandGroup {

  /**
   * Creates a new ArmToPresetBackwardSequence. This will be used immediately after scoring to reset 
   * to the folded position. It rotates the shoulder back while lifting the elbow up to avoid 
   * catching on the pole, moves the elbow down, and then moves the shoulder back upright
   * @param turretAngle
   * @param shoulderAngle
   * @param elbowAngle
   * @param wristAngle
   */
  public ConeHighPostCenterSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //TODO: figure out exactly what operators want and reprogram to match

    addCommands(
      new ElbowExtendPreset(-133),
      new ArmToPresetNoTurret(42, -113, -144)
    );
  }

  public static void registerWithTestingDashboard() {
    Arm arm = Arm.getInstance();
    ConeHighPostCenterSequence cmd = new ConeHighPostCenterSequence();
    TestingDashboard.getInstance().registerCommand(arm, "Automatic", cmd);
  }
}
