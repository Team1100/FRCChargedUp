// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.testingdashboard.TestingDashboard;

public class Vision extends SubsystemBase {
  private static Vision m_vision;
  private static int aprilTagTarget;
  NetworkTable m_Ntable;

  PowerDistribution m_pDBoard;

  /**
   * Creates a new Vision.
   */
  private Vision() {
    m_Ntable = NetworkTableInstance.getDefault().getTable("Shuffleboard/Vision");
    m_pDBoard =  new PowerDistribution();
    aprilTagTarget = 0;
  }

  public static Vision getInstance() {
    if (m_vision == null) {
      m_vision = new Vision();
      TestingDashboard.getInstance().registerSubsystem(m_vision, "Vision");

      Shuffleboard.getTab("Vision")
          .add("aprilTagTargetID", 0);
      
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
    return m_Ntable.getEntry("offset").getDouble(0);
  }

  public double getTargetYaw() {
    return m_Ntable.getEntry("robotYaw").getDouble(0);
  }

  public boolean isAprilTagTargetFound() {
    if (m_Ntable.getEntry("aprilTagTargetDetected").getDouble(0) == 0)
      return false;
    else
      return true;
  }

  public void enableLimeLight(boolean enable) {
    m_pDBoard.setSwitchableChannel(enable);
  }

  public void setTargetAprilTag(int apriltagID) {
    aprilTagTarget = apriltagID;
    m_Ntable.getEntry("aprilTagTargetID").setInteger(aprilTagTarget);
  }

  public int getTargetAprilTag() {
    return aprilTagTarget;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
