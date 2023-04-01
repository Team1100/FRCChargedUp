// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;

/** Class that sets up PID Velocity control for a CANSparkMax
 * Extends CANSparkMax and overrides set to set a target velocity
 * Designed to take advantage of wpilib DifferentialDrive class with velocity PID control 
 */
public class VelocityDriveSparkMax extends CANSparkMax
{
    private double m_P, m_I, m_D, m_FF, m_Iz;
    private SparkMaxPIDController m_PidController;
    private DriveMode m_driveMode;

    public enum DriveMode 
    {
        kPower, //Normal duty cycle based drive
        kPIDVelocity //Translates to PID Velocity
    }

    public VelocityDriveSparkMax(int deviceId, MotorType type, double P, double I, double D)
    {
        super(deviceId, type);
        super.setClosedLoopRampRate(Constants.DRIVE_RAMP_RATE);
        m_PidController = super.getPIDController();
        m_PidController.setOutputRange(-Constants.DRIVE_CLOSED_LOOP_MAX_OUTPUT, Constants.DRIVE_CLOSED_LOOP_MAX_OUTPUT);
        setPID(P, I, D);
        m_driveMode = DriveMode.kPower;
    }

    public void setPID(double P, double I, double D)
    {
        setPID(P, I, D, 0, 0);
    }

    public void setPID(double P, double I, double D, double FF, double Iz)
    {
        if(P != m_PidController.getP()){m_P=P; m_PidController.setP(m_P);}
        if(I != m_PidController.getI()){m_I=I; m_PidController.setI(m_I);}
        if(D != m_PidController.getD()){m_D=D; m_PidController.setD(m_D);}
        if(FF != m_PidController.getFF()){m_FF=FF; m_PidController.setFF(m_FF);}
        if(Iz != m_PidController.getIZone()){m_Iz=Iz; m_PidController.setIZone(m_Iz);}
    }

    public DriveMode getDriveMode(){return m_driveMode;}

    public void setDriveMode(DriveMode mode)
    {
        m_driveMode = mode;
    }

    @Override
    public void set(double speed)
    {
        if(m_driveMode == DriveMode.kPIDVelocity)
        {
            double rpm = speed * Constants.DRIVE_MAX_MOTOR_RPM;
            System.out.println("Driving in Velocity PID mode at " + rpm + "rpm, from speed "+ speed + ", P:"+m_PidController.getP());
            m_PidController.setReference(rpm, ControlType.kVelocity);
        }
        else
        {
            super.set(speed);
        }
    }

}
