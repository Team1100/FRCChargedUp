// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Hand;
import frc.robot.testingdashboard.TestingDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartIntakeCube extends ParallelDeadlineGroup {
  /** Creates a new SmartExpelCone. */
  public SmartIntakeCube() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new CheckForCurrentSpike());
    addCommands(new IntakeCube(true));
  }

    //Register with TestingDashboard
    public static void registerWithTestingDashboard() {
      Hand hand = Hand.getInstance();
      SmartIntakeCube cmd = new SmartIntakeCube();
      TestingDashboard.getInstance().registerCommand(hand, "Basic", cmd);
    }
}
