// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.Hand.IntakeCubeTimed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimedFloorPickup extends ParallelDeadlineGroup {
  /** Creates a new TimedFloorPickup. */
  public TimedFloorPickup() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new IntakeCubeTimed());
    addCommands(new DriveDistance(-36, 0.1, 0.1, 0, true));
  }
}
