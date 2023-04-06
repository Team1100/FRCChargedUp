// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.Constants;
import frc.robot.helpers.VelocityDriveSparkMax.DriveMode;
import frc.robot.subsystems.Drive;

public class AutoBalance {
    //private ADIS16470_IMU m_gyro;
    private BuiltInAccelerometer mRioAccel;
    private int state;
    private int debounceCount;
    private boolean dropDetected;
    private double robotSpeedSlow;
    private double robotSpeedFast;
    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;
    private double singleTapTime;
    private double scoringBackUpTime;
    private double doubleTapTime;
    private double onPlatformROC;
    boolean m_isFinished;
    final boolean USE_GYRO_ALL;


    /**
     * State Machine used to balance the robot on the charging station
     * @param direction the direction that the robot is driving. 
     */
    public AutoBalance(int direction) {
        mRioAccel = new BuiltInAccelerometer();

        USE_GYRO_ALL = false;

        // m_gyro = Drive.m_gyro;

        state = 0;
        debounceCount = 0;

        /**********
         * CONFIG *
         **********/
        // Speed the robot drives while scoring/approaching station, default = 0.4
        robotSpeedFast = 0.53 * direction;

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        // default = 0.2
        robotSpeedSlow = 0.4 * direction;

        // Angle where the robot knows it is on the charge station, default = 14.5
        onChargeStationDegree = 8 * direction;

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence as well as for auto balancing,
        // default = 10.0
        levelDegree = 7;

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noise, but too high can make the auto run
        // slower, default = 0.15
        debounceTime = 0.1;

        // Amount of time to drive towards to scoring target when trying to bump the
        // game piece off
        // Time it takes to go from starting position to hit the scoring target
        singleTapTime = 0.4;

        // Amount of time to drive away from knocked over gamepiece before the second
        // tap
        scoringBackUpTime = 0.2;

        // Amount of time to drive forward to secure the scoring of the gamepiece
        doubleTapTime = 0.3;

        // Rate of change of the angle for the robot to assume it is on top of the platform
        // and stop; Calculated by the derivative of the angle measurements v. time.
        onPlatformROC = 0;

        m_isFinished = false;
    }

    public double getPitch() {
        return Math.atan2((-mRioAccel.getX()),
                Math.sqrt(mRioAccel.getY() * mRioAccel.getY() + mRioAccel.getZ() * mRioAccel.getZ())) * 57.3;
    }

    public double getRoll() {
        return Math.atan2(mRioAccel.getY(), mRioAccel.getZ()) * 57.3;
    }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio


    public double getTilt() {
        if(Constants.D_ENABLE_GYRO_BALANCE) {
            // Jack and Dan's alternate sensor readings, as of 3/30/23
            return Drive.m_gyro.getAngle();
        } else {
        double pitch = getPitch();
        double roll = getRoll();
        if ((pitch + roll) >= 0) {
            return Math.sqrt(pitch * pitch + roll * roll);
        } else {
            return -Math.sqrt(pitch * pitch + roll * roll);
        }
      }
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    } 

    // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0, which left and right motors should be set
    // to.
    public double backwardAutoBalanceRoutine() {

        Drive.getInstance().updateRioTiltAverages();
        switch (state) {
            // drive forwards to approach station, exit when tilt is detected
            case 0:
                if (Drive.getInstance().getTotalAverageRioAccel() > Math.abs(onChargeStationDegree)) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 1;
                    debounceCount = 0;
                    Drive.getInstance().setDriveMode(DriveMode.kPIDVelocity);
                    return -robotSpeedSlow;
                }
                dropDetected = false;
                return -robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 1:
                if (dropDetected) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    if(Drive.m_gyro.getAngle() < levelDegree && Drive.m_gyro.getAngle() > -levelDegree)
                    {
                        state = 2;
                        debounceCount = 0;
                        return 0;
                    }
                    else
                    {
                        debounceCount = 0;
                        dropDetected = false;
                        return -robotSpeedSlow;
                    }
                }
                if (Drive.m_gyro.getRate() <= -6) {
                    dropDetected = true;
                    return 0;
                }
                return -robotSpeedSlow;
            // on charge station, stop motors and wait for end of auto
            case 2:

            if(!Constants.D_ENABLE_GYRO_BALANCE && !USE_GYRO_ALL) {
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
            }
                //Shortcut to using the Gyro (if enabled):
            if(Constants.D_ENABLE_GYRO_BALANCE && USE_GYRO_ALL) {
                if (Drive.m_gyro.getAngle() < levelDegree && Drive.m_gyro.getAngle() > -levelDegree) {
                    debounceCount++;
                }
            }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 3;
                    debounceCount = 0;
                    m_isFinished = true;
                    return 0;
                }

                if (Drive.getInstance().m_gyro.getAngle() >= levelDegree) {
                    return -robotSpeedSlow * 0.75;
                } else if (Drive.getInstance().m_gyro.getAngle() <= -levelDegree) {
                    return robotSpeedSlow * 0.75;
                }
            case 3:
                return 0;
        }
        return 0;
    }

        // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0, which left and right motors should be set
    // to.
    public double forwardAutoBalanceRoutine() {
        Drive.getInstance().updateRioTiltAverages();
        switch (state) {
            // drive forwards to approach station, exit when tilt is detected
            case 0:
                if (Drive.getInstance().getTotalAverageRioAccel() < onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 1;
                    debounceCount = 0;
                    Drive.getInstance().setDriveMode(DriveMode.kPIDVelocity);
                    return -robotSpeedSlow;
                }
                dropDetected = false;
                return -robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 1:
                if (dropDetected) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    if(Drive.m_gyro.getAngle() < levelDegree && Drive.m_gyro.getAngle() > -levelDegree)
                    {
                        state = 2;
                        debounceCount = 0;
                        return -0.2;
                    }
                    else
                    {
                        debounceCount = 0;
                        dropDetected = false;
                        return -robotSpeedSlow;
                    }
                }      // Was usually 7deg/s  \/
                if (Drive.m_gyro.getRate() >= 6) {
                    dropDetected = true;
                    return -0.2;
                }
                return -robotSpeedSlow;
            // on charge station, stop motors and wait for end of auto
            case 2:

            if(!Constants.D_ENABLE_GYRO_BALANCE && !USE_GYRO_ALL) {
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
            }
                //Shortcut to using the Gyro (if enabled):
            if(Constants.D_ENABLE_GYRO_BALANCE && USE_GYRO_ALL) {
                if (Drive.m_gyro.getAngle() < levelDegree && Drive.m_gyro.getAngle() > -levelDegree) {
                    debounceCount++;
                }
            }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 3;
                    debounceCount = 0;
                    m_isFinished = true;
                    return 0;
                }

                if (Drive.getInstance().m_gyro.getAngle() >= levelDegree) {
                    return robotSpeedSlow * 0.75;
                } else if (Drive.getInstance().m_gyro.getAngle() <= -levelDegree) {
                    return -robotSpeedSlow * 0.75;
                }
            case 3:
                return 0;
        }
        return 0;
    }

    public void clearFinished() {
        state = 0;
        debounceCount = 0;
        m_isFinished = false;
    }

    public boolean isFinished() {
        return m_isFinished;
    }
}