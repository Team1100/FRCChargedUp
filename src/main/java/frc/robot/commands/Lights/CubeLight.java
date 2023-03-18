// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.testingdashboard.TestingDashboard;

public class CubeLight extends CommandBase {
  /** Creates a new cubeLight. */
  Lights m_lights;

  public CubeLight() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lights = Lights.getInstance();
    addRequirements(m_lights);
  }

  public static void registerWithTestingDashboard() {
    Lights lights = Lights.getInstance();
    CubeLight cmd = new CubeLight();
    TestingDashboard.getInstance().registerCommand(lights, "Lights", cmd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lights.enableCubeLight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
