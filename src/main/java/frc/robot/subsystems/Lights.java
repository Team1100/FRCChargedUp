// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TestingDashboard;

public class Lights extends SubsystemBase {

  private DigitalOutput m_leftLED;
  private DigitalOutput m_rightLED;

  private static Lights m_lights;

  private boolean blinking;
  private int counter;

  /** Creates a new Lights. */
  public Lights() {
    m_leftLED = new DigitalOutput(RobotMap.L_LEFT_LED);
    m_rightLED = new DigitalOutput(RobotMap.L_RIGHT_LED);

    blinking = false;
    counter = 0;
  }

  public static Lights getInstance() {
    if (m_lights == null) {
      m_lights = new Lights();

      TestingDashboard.getInstance().registerSubsystem(m_lights, "lights");
    }
    return m_lights;
  }

  public void enableLights() {
    m_leftLED.set(true);
    m_rightLED.set(true);
  }

  public void disableLights() {
    m_leftLED.set(false);
    m_rightLED.set(false);
  }

  public void enableConeLight() {
    m_leftLED.set(true);
    m_rightLED.set(false);
  }

  public void enableCubeLight() {
    m_leftLED.set(false);
    m_rightLED.set(true);
  }

  public void blinkLights() {
    blinking = true;
  }

  public DigitalOutput getLeftLED() {
    return m_leftLED;
  }

  public DigitalOutput getRightLED() {
    return m_rightLED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    if(blinking == true) {
      if(((counter % 20) == 0)) {
        m_lights.enableCubeLight();
      } 
      if(((counter % 20) == 10)) {
        m_lights.enableConeLight();
      } 
    }
    
    if(counter > 100) {
      blinking = false;
      disableLights();
      counter = 0;
    } else if(blinking == true) {
      counter++;
    }

  }
}
