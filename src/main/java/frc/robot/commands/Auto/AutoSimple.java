// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ArmToHomePosition;
import frc.robot.commands.Arm.ZeroArmEncoders;
import frc.robot.commands.Arm.sequences.ConeFarPostStraightSequence;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.Hand.ExpelCone;
import frc.robot.subsystems.Auto;
import frc.robot.testingdashboard.TestingDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSimple extends SequentialCommandGroup {
  /** Creates a new AutoSimple. */
  public AutoSimple() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ZeroArmEncoders(),
      new ConeFarPostStraightSequence(),
      new ExpelCone(true),
      new DriveDistance(-10, 0.3, 0.3, 1, true),
      new ArmToHomePosition()
    );
  }

  public static void registerWithTestingDashboard() {
    AutoSimple cmd = new AutoSimple();
    Auto auto = Auto.getInstance();
    TestingDashboard.getInstance().registerCommand(auto, "Sequences", cmd);
  }
}
