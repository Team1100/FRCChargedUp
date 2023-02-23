package frc.robot.helpers;
import java.awt.geom.Point2D;
import frc.robot.helpers.Point3D;
import frc.robot.subsystems.Arm;


public class ArmSegmentHelper {
    /**
     * The cooridinate system used in this class has the origin at the bottom center of the robot (the floor is at z = 0)
     * The x-axis runs along the length of the robot (parallel to the sides), with positive being the front
     * the y-axis runs along the width of the robot (parallel to the front and back), with positive being the right
     * the z=axis is the vertical axis, running along the height of the robot, with positive being up
     * 
     * The shoulder angle is measured from the horizontal (flat is 0 degrees)
     * the forearm angle is measured relative the the shoulder angle (currently zero is folded down)
     */

    // TODO: replace with accurate values
    public static final double SHOULDER_LENGTH = 2; // in feet
    public static final double FOREARM_LENGTH = 1.4; // in feet
    public static final Point3D SHOULDER_JOINT_COOR = new Point3D(0,0,1);
    // Used for the overextention prevention
    // TODO: replace with accurate values
    public static final double ROBOT_WIDTH = 2;
    public static final double ROBOT_LENGTH = 3;

    public static final double XY_EXTENTION_MAX = 4; // from bumper
    public static final double FRONT_EXTENTION_LIMIT = ROBOT_LENGTH/2 + XY_EXTENTION_MAX;
    public static final double BACK_EXTENTION_LIMIT = -FRONT_EXTENTION_LIMIT;
    public static final double RIGHT_EXTENTION_LIMIT = ROBOT_WIDTH/2 + XY_EXTENTION_MAX;
    public static final double LEFT_EXTENTION_LIMIT = -FRONT_EXTENTION_LIMIT;
    public static final double UPPER_EXTENTION_LIMIT = 6.5; // measured from floor
    public static final double LOWER_EXTENTION_LIMIT = 1; // TODO: Fill in with lower limit that arm should not cross (measured from the floor)

    // This constant indicates the conversion factor between Rotations per Minute to Radians per Second
    public static final double RPM_TO_RAD = 2 * Math.PI / 60;

    public static final double LOOK_AHEAD_TIME = .25; // seconds

    
    private Point3D m_handCoor;

    Arm m_arm;

    public enum extentionLimitFaces {
        FRONT,
        BACK,
        LEFT,
        RIGHT,
        BOTTOM,
        TOP,
        NONE
    }
    
    public ArmSegmentHelper() {
        m_arm = Arm.getInstance();
        // TODO: Initialize with a more accurate coordinate.
        // Initially has m_handCoor set as if the arm is straight up with the forearm folded down, may want to i
        m_handCoor = new Point3D(0,0, SHOULDER_LENGTH - FOREARM_LENGTH + SHOULDER_JOINT_COOR.getZ());
    }

    /**
     * This method calculates the 3D coordinate of the hand based on the current joint angles.
     * @return A Point3D object representing the 3D cooridnate of the hand
     */
    public Point3D calculateHandCoor() {
        // TODO: Check origin of angle measurment and adjust to match with coordinate system
        double x = SHOULDER_JOINT_COOR.getX() + Math.cos(Math.toRadians(m_arm.getTurretAngle())) *
            (SHOULDER_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle() + 180)));
            

        double y = SHOULDER_JOINT_COOR.getY() + Math.sin(Math.toRadians(m_arm.getTurretAngle())) *
            (SHOULDER_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle() + 180)));


        double z = SHOULDER_JOINT_COOR.getZ() 
            + SHOULDER_LENGTH * Math.sin(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.sin(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle() + 180));

        m_handCoor = new Point3D(x,y,z);
        return m_handCoor;
    }

    /**
     * This method calculates the velocity of the hand based on the current angles and velocities of the joints
     * @return A Point3D object representing the 3D velocity vector of the hand.
     */
    public Point3D calculateHandVelocity() { // returns measurement in feet per second
        // TODO: Test if hand velocity calculations are accurate. May want to take derivative of the equation that calculates the coordinates.
        // TODO: Check origin of angle measurment and adjust to match with coordinate system
        double XYDistanceFromJointToHand = (SHOULDER_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle() + 180)));
        // Velocity contributed by the rotation of the turret
        double turretVelX = Math.sin(m_arm.getTurretAngle()) * m_arm.getTurretVelocity() * XYDistanceFromJointToHand;
        double elbowVelX = (m_arm.getShoulderVelocity() * RPM_TO_RAD * SHOULDER_LENGTH * Math.sin(Math.toRadians(m_arm.getShoulderAngle())));
        // Adds velocity contributed by arm segments and turret rotation together
        double handVelX = turretVelX + Math.cos(Math.toRadians(m_arm.getTurretAngle()))*(elbowVelX + (m_arm.getElbowVelocity() * RPM_TO_RAD * FOREARM_LENGTH * Math.sin(Math.toRadians(m_arm.getElbowAngle()))));

        double turretVelY = Math.cos(m_arm.getTurretAngle()) * m_arm.getTurretVelocity() * XYDistanceFromJointToHand;
        double elbowVelY = (m_arm.getShoulderVelocity() * RPM_TO_RAD * SHOULDER_LENGTH * Math.sin(Math.toRadians(m_arm.getShoulderAngle())));
        // Adds velocity contributed by arm segments and turret rotation together
        double handVelY = turretVelY + Math.sin(Math.toRadians(m_arm.getTurretAngle()))*(elbowVelY + (m_arm.getElbowVelocity() * RPM_TO_RAD * FOREARM_LENGTH * Math.sin(Math.toRadians(m_arm.getElbowAngle()))));

        double elbowVelZ = (m_arm.getShoulderVelocity() * RPM_TO_RAD * SHOULDER_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle())));
        // Adds velocity contributed by both arm segments
        double handVelZ = elbowVelZ + (m_arm.getElbowVelocity() * RPM_TO_RAD * FOREARM_LENGTH * Math.cos(Math.toRadians(m_arm.getElbowAngle())));
        
        return new Point3D(handVelX,handVelY,handVelZ);
    }

    /**
     * This method takes an input coordinate and tests whether it has is outside the boundries of the arm movement
     * @param testCoor A Point3D object representing the coordinate to be tested
     * @return         An enum indicating the side of the robot the robot has overextended
     */
    public extentionLimitFaces checkForOverextention(Point3D testCoor) {
        // TODO: If overextended, return Minimum Translation Vector (the smallest vector required to translate the hand back into legal space)
        if (testCoor.getX() >= FRONT_EXTENTION_LIMIT) {
            return extentionLimitFaces.FRONT;
        } else if (testCoor.getX() <= BACK_EXTENTION_LIMIT) {
            return extentionLimitFaces.BACK;
        }

        if (testCoor.getY() >= RIGHT_EXTENTION_LIMIT) {
            return extentionLimitFaces.RIGHT;
        } else if (testCoor.getY() <= LEFT_EXTENTION_LIMIT) {
            return extentionLimitFaces.LEFT;
        }

        if (testCoor.getZ() >= UPPER_EXTENTION_LIMIT) {
            return extentionLimitFaces.TOP;
        } else if (testCoor.getZ() <= LOWER_EXTENTION_LIMIT) {
            return extentionLimitFaces.BOTTOM;
        }
        return extentionLimitFaces.NONE;
    }

    public Point3D calculateFutureCoor(double lookAheadTime) {
        // TODO: Track the elbow joint coordinate
        // Currently only looks at the coordinate and velocity of the hand.
        calculateHandCoor();
        Point3D m_currentVelocity = calculateHandVelocity();
        return new Point3D(
            m_handCoor.getX() + m_currentVelocity.getX() * lookAheadTime, 
            m_handCoor.getY() + m_currentVelocity.getY() * lookAheadTime, 
            m_handCoor.getZ() + m_currentVelocity.getZ() * lookAheadTime);
    }

    /**
     * 
     * @param lookAheadTime The time used to predict the distance moved. A value of zero checks for an overextention based on the current coordinate
     * @return              An enum indicating the side of the robot the robot has overextended
     * 
     */
    public extentionLimitFaces predictOverExtention(double lookAheadTime) {
        return checkForOverextention(calculateFutureCoor(lookAheadTime));
    }
}
