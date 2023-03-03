// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.testingdashboard.TestingDashboard;

public class Vision extends SubsystemBase {
  private static Vision m_vision;
  NetworkTable table;

  /**
   * Creates a new Vision.
   */
  private Vision() {
    table = NetworkTableInstance.getDefault().getTable("Shuffleboard/Vision");
  }

  public static Vision getInstance() {
    if (m_vision == null) {
      m_vision = new Vision();
      TestingDashboard.getInstance().registerSubsystem(m_vision, "Vision");
      
      Shuffleboard.getTab("Vision")
          .add("hueMin", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("hueMax", 255)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("satMin", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("satMax", 255)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("valMin", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("valMax", 255)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
    }
    return m_vision;
  }

  public double getTargetOffset() {
    return table.getEntry("offset").getDouble(0);
  }



  public boolean isTargetFound() {
    if (table.getEntry("targetDetected").getDouble(0) == 0)
      return false;
    else
      return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
