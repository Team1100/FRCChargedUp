// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hand;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Auto.Wait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExpelConeTimed extends ParallelDeadlineGroup {
  /** Creates a new ExpelConeTimed. */
  public ExpelConeTimed() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new Wait(.6, true));
    addCommands(new ExpelCone(true));
  }
}
