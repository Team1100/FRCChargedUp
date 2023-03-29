// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotMap;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.testingdashboard.TestingDashboard;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Hand extends SubsystemBase {

  private CANSparkMax m_handMotor;

  private static Hand m_hand;

  ArrayList<Double> m_hand_motor_current_values;
  public static final int HAND_MOTOR_CURRENT_INITIAL_CAPACITY = 20; // This is 1000 miliseconds divided in 20 millisecond chunks


  /** Creates a new Claw. */
  private Hand() {

    m_handMotor = new CANSparkMax(RobotMap.H_MOTOR, MotorType.kBrushless);

    m_handMotor.restoreFactoryDefaults();

    m_handMotor.setIdleMode(IdleMode.kBrake);

    // initialize motor current variables
    m_hand_motor_current_values = new ArrayList<Double>(HAND_MOTOR_CURRENT_INITIAL_CAPACITY);
    for (int i = 0; i < HAND_MOTOR_CURRENT_INITIAL_CAPACITY; i++) {
      m_hand_motor_current_values.add(0.0);
    }

  }

  public static Hand getInstance() {
    if (m_hand == null) {
      m_hand = new Hand();
      TestingDashboard.getInstance().registerSubsystem(m_hand, "Hand");
      TestingDashboard.getInstance().registerNumber(m_hand, "MotorInput", "HandPower", Constants.DEFAULT_INTAKE_CUBE_POWER);
      TestingDashboard.getInstance().registerNumber(m_hand, "MotorOutput", "HandOutputCurrent", 0.0d);
    }
    return m_hand;
  }

  void updateMotorCurrentAverages() {
    double handMotorCurrent = m_handMotor.getOutputCurrent();
    m_hand_motor_current_values.add(handMotorCurrent);


    // Trim current buffers until they contain the correct number of entries.
    // Old entries are removed first.
    while (m_hand_motor_current_values.size() > HAND_MOTOR_CURRENT_INITIAL_CAPACITY) {
      m_hand_motor_current_values.remove(0);
    }

  }

  public double getTotalAverageHandCurrent() {
    return arrayListAverage(m_hand_motor_current_values);
  }

  public static double arrayListAverage(ArrayList<Double> arrayList) {
    double sum = 0;
    for (int i = 0; i < arrayList.size(); i++) {
      sum += arrayList.get(i);
    }
    return sum / arrayList.size();
  }


  public void setHandMotorPower(double value) {
    m_handMotor.set(value);
  }

  @Override
  public void periodic() {
    updateMotorCurrentAverages();
    // This method will be called once per scheduler run
    TestingDashboard.getInstance().updateNumber(m_hand, "HandOutputCurrent", getTotalAverageHandCurrent());
  }
}
