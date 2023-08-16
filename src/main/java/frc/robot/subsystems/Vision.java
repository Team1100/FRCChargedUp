// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testingdashboard.TestingDashboard;

public class Vision extends SubsystemBase {
  private static Vision m_vision;
  private static int aprilTagTarget;
  private static int detectionMode;
  private static double[] defaultHSV;

  NetworkTable m_Ntable;

  PowerDistribution m_pDBoard;
  GenericEntry detectionEntry;
  GenericEntry tagNumEntry;
  GenericEntry camInUseEntry;
  GenericEntry colorDetectionConstSub;
  
  public static final int DETECTING_NOTHING = 0;
  public static final int DETECTING_COLOR = 1;
  public static final int DETECTING_APRILTAG = 2;

  public static final int CAMERA_1 = 1;
  public static final int CAMERA_2 = 2;

  /**
   * Creates a new Vision.
   */
  private Vision() {
    m_Ntable = NetworkTableInstance.getDefault().getTable("Shuffleboard/Vision");

    // Array of constants used to identify the target using color detection (Color Detection Constants)
    // {idealAreaRatio, areaTolerance, idealApectRatio, aspectTolerance, idealYCoor, yCoorTolerance, idealXCoor, xCoorTolerance, minBoundingArea}
    double[] colorDetectConstants = {1, 1, 2, 3, -1, 200, -1, 200, 110};
    // HSV Constants
    // {hueMin, hueMax, satMin, satMax, valMin, valMax}
    defaultHSV = new double[] {109.92, 153, 103, 255, 103, 255};


    m_pDBoard =  new PowerDistribution();
    aprilTagTarget = 0;
    detectionMode = 0;
    detectionEntry = Shuffleboard.getTab("Vision").add("detectionMode", DETECTING_COLOR).getEntry();
    camInUseEntry = Shuffleboard.getTab("Vision").add("cameraInUse", CAMERA_1).getEntry();
    tagNumEntry = Shuffleboard.getTab("Vision").add("aprilTagTargetID", 1).getEntry();
    colorDetectionConstSub = Shuffleboard.getTab("Vision").add("colorDetectConst", colorDetectConstants).getEntry();

  }

  public static Vision getInstance() {
    if (m_vision == null) {
      m_vision = new Vision();
      TestingDashboard.getInstance().registerSubsystem(m_vision, "Vision");

      SmartDashboard.putNumber("Target AprilTag ID", 1);
      
      Shuffleboard.getTab("Vision")
          .add("hueMin", defaultHSV[0])
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("hueMax", defaultHSV[1])
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("satMin", defaultHSV[2])
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("satMax", defaultHSV[3])
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("valMin", defaultHSV[4])
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
      Shuffleboard.getTab("Vision")
          .add("valMax", defaultHSV[5])
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 255)) // specify widget properties here
          .getEntry();
    }
    return m_vision;
  }

  public double getTargetOffset() {
    double offset = m_Ntable.getEntry("offset").getDouble(0);
    SmartDashboard.putNumber("offset", offset);
    return offset;
  }

  /**
   * This only returns a valid 
   * @return The angle of the target with respect to the central axis of the camera
   */
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
    tagNumEntry.setInteger(aprilTagTarget);
  }

  /**
   * 
   * @param detectMode Determines the method of detection. Options are detecting nothing, detecting objects based on color, or detecting apriltags.
   * Can input parameters with Vision constants.
   */
  public void setDetectionMode(int detectMode) {
    detectionMode = detectMode;
    detectionEntry.setInteger(detectMode);
  }

  /**
   * 
   * @param camInUse Determines which camera to use, if multiple cameras are needed on the robot. This allows for switching between cameras.
   * Can input parameters with Vision constants.
   */
  public void setCamera(int camInUse) {
    camInUseEntry.setInteger(camInUse);
  }


  public int getTargetAprilTag() {
    return aprilTagTarget;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double offset = m_Ntable.getEntry("offset").getDouble(-1000);
    SmartDashboard.putNumber("offset", offset);
    setTargetAprilTag((int)SmartDashboard.getNumber("Target AprilTag ID", 1));
  }
}
