package frc.robot.helpers;
import java.awt.geom.Point2D;
import frc.robot.Vector;
import frc.robot.subsystems.Arm;
import frc.robot.testingdashboard.TestingDashboard;


public class ArmSegmentHelper {
    /**
     * The cooridinate system used in this class has the origin at the bottom center of the robot (the floor is at z = 0)
     * The x-axis runs along the length of the robot (parallel to the sides), with positive being the front
     * the y-axis runs along the width of the robot (parallel to the front and back), with positive being the right
     * the z-axis is the vertical axis, running along the height of the robot, with positive being up
     * 
     * The shoulder angle is measured from the horizontal (flat is 0 degrees)
     * the forearm angle is measured relative the the shoulder angle (currently zero is folded down)
     */

    
    public static final double SHOULDER_LENGTH = 40; // in inches
    public static final double FOREARM_LENGTH = 30; // in inches
    // TODO: replace with accurate value(s):
    // TODO: Fill in with accurte values for location of shoulder joint
    public static final Vector SHOULDER_JOINT_COOR = new Vector(0,0,14.5);
    // Used for the overextention prevention
    // TODO: replace with accurate values
    public static final double ROBOT_WIDTH = 33;
    public static final double ROBOT_LENGTH = 39;

    public static final double XY_EXTENTION_MAX = 48; // from bumper
    public static final double FRONT_EXTENTION_LIMIT = ROBOT_LENGTH/2 + XY_EXTENTION_MAX;
    public static final double BACK_EXTENTION_LIMIT = -FRONT_EXTENTION_LIMIT;
    public static final double RIGHT_EXTENTION_LIMIT = ROBOT_WIDTH/2 + XY_EXTENTION_MAX;
    public static final double LEFT_EXTENTION_LIMIT = -FRONT_EXTENTION_LIMIT;
    public static final double UPPER_EXTENTION_LIMIT = 6.5 * 12; // measured from floor
    public static final double LOWER_EXTENTION_LIMIT = 24; // TODO: Fill in with lower limit that arm should not cross (measured from the floor)

    // This constant indicates the conversion factor between Rotations per Minute to Radians per Second
    public static final double RPM_TO_RAD = 2 * Math.PI / 60;
    public static final double LOOK_AHEAD_TIME = .25; // seconds

    
    private Vector m_handCoor;
    private Vector m_handVel;

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
        // TODO: Initialize with a more accurate coordinate.
        // Initially has m_handCoor set as if the arm is straight up with the forearm folded down
        m_handCoor = new Vector(0,0, SHOULDER_LENGTH - FOREARM_LENGTH + SHOULDER_JOINT_COOR.z);
        m_handVel = new Vector(0,0,0);
    }

    public void updateArmSegmentValues() {
        calculateHandCoor();
        calculateHandVelocity();
    }

    /**
     * This method calculates the 3D coordinate of the hand based on the current joint angles.
     * @return A Vector object representing the 3D cooridnate of the hand
     */
    public Vector calculateHandCoor() {
        Arm arm = Arm.getInstance();
        // TODO: Check origin of angle measurment and adjust to match with coordinate system
        double x = SHOULDER_JOINT_COOR.x + Math.cos(Math.toRadians(arm.getTurretAngle())) *
            (SHOULDER_LENGTH * Math.cos(Math.toRadians(arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(arm.getShoulderAngle() + arm.getElbowAngle() + 180)));
            

        double y = SHOULDER_JOINT_COOR.y + Math.sin(Math.toRadians(arm.getTurretAngle())) *
            (SHOULDER_LENGTH * Math.cos(Math.toRadians(arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(arm.getShoulderAngle() + arm.getElbowAngle() + 180)));


        double z = SHOULDER_JOINT_COOR.z 
            + SHOULDER_LENGTH * Math.sin(Math.toRadians(arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.sin(Math.toRadians(arm.getShoulderAngle() + arm.getElbowAngle() + 180));

        TestingDashboard.getInstance().updateNumber(arm, "HandXCoor", x);
        TestingDashboard.getInstance().updateNumber(arm, "HandYCoor", y);
        TestingDashboard.getInstance().updateNumber(arm, "HandZCoor", z);

        m_handCoor.setValues(x,y,z);
        return new Vector(x, y, z);
    }

    /**
     * This method calculates the velocity of the hand based on the current angles and velocities of the joints
     * @return A Vector object representing the 3D velocity vector of the hand.
     */
    public Vector calculateHandVelocity() { // returns measurement in inches per second
        Arm arm = Arm.getInstance();
        // TODO: Test if hand velocity calculations are accurate. May want to take derivative of the equation that calculates the coordinates.
        // TODO: Check origin of angle measurment and adjust to match with coordinate system
        double XYDistanceFromJointToHand = (SHOULDER_LENGTH * Math.cos(Math.toRadians(arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(arm.getShoulderAngle() + arm.getElbowAngle() + 180)));
        // Velocity contributed by the rotation of the turret
        double turretVelX = Math.sin(arm.getTurretAngle()) * arm.getTurretVelocity() * XYDistanceFromJointToHand;
        double elbowVelX = (arm.getShoulderVelocity() * RPM_TO_RAD * SHOULDER_LENGTH * Math.sin(Math.toRadians(arm.getShoulderAngle())));
        // Adds velocity contributed by arm segments and turret rotation together
        double handVelX = turretVelX + Math.cos(Math.toRadians(arm.getTurretAngle()))*(elbowVelX + (arm.getElbowVelocity() * RPM_TO_RAD * FOREARM_LENGTH * Math.sin(Math.toRadians(arm.getElbowAngle()))));

        double turretVelY = Math.cos(arm.getTurretAngle()) * arm.getTurretVelocity() * XYDistanceFromJointToHand;
        double elbowVelY = (arm.getShoulderVelocity() * RPM_TO_RAD * SHOULDER_LENGTH * Math.sin(Math.toRadians(arm.getShoulderAngle())));
        // Adds velocity contributed by arm segments and turret rotation together
        double handVelY = turretVelY + Math.sin(Math.toRadians(arm.getTurretAngle()))*(elbowVelY + (arm.getElbowVelocity() * RPM_TO_RAD * FOREARM_LENGTH * Math.sin(Math.toRadians(arm.getElbowAngle()))));

        double elbowVelZ = (arm.getShoulderVelocity() * RPM_TO_RAD * SHOULDER_LENGTH * Math.cos(Math.toRadians(arm.getShoulderAngle())));
        // Adds velocity contributed by both arm segments
        double handVelZ = elbowVelZ + (arm.getElbowVelocity() * RPM_TO_RAD * FOREARM_LENGTH * Math.cos(Math.toRadians(arm.getElbowAngle())));
        
        TestingDashboard.getInstance().updateNumber(arm, "HandXVel", handVelX);
        TestingDashboard.getInstance().updateNumber(arm, "HandYVel", handVelY);
        TestingDashboard.getInstance().updateNumber(arm, "HandZVel", handVelZ);

        m_handVel.setValues(handVelX, handVelY, handVelZ);
        return new Vector(handVelX,handVelY,handVelZ);
    }

    /**
     * This method takes an input coordinate and tests whether it has is outside the boundries of the arm movement
     * @param testCoor A Vector object representing the coordinate to be tested
     * @return         An enum indicating the side of the robot the robot has overextended
     */
    public extentionLimitFaces checkForOverextention(Vector testCoor) {
        // TODO: If overextended, return Minimum Translation Vector (the smallest vector required to translate the hand back into legal space)
        if (testCoor.x >= FRONT_EXTENTION_LIMIT) {
            return extentionLimitFaces.FRONT;
        } else if (testCoor.x <= BACK_EXTENTION_LIMIT) {
            return extentionLimitFaces.BACK;
        }

        if (testCoor.y >= RIGHT_EXTENTION_LIMIT) {
            return extentionLimitFaces.RIGHT;
        } else if (testCoor.y <= LEFT_EXTENTION_LIMIT) {
            return extentionLimitFaces.LEFT;
        }

        if (testCoor.z >= UPPER_EXTENTION_LIMIT) {
            return extentionLimitFaces.TOP;
        } else if (testCoor.z <= LOWER_EXTENTION_LIMIT) {
            return extentionLimitFaces.BOTTOM;
        }
        return extentionLimitFaces.NONE;
    }

    public Vector calculateFutureCoor(double lookAheadTime) {
        // TODO: Track the elbow joint coordinate
        // Currently only looks at the coordinate and velocity of the hand.
        calculateHandCoor();
        Vector m_currentVelocity = calculateHandVelocity();
        return new Vector(
            m_handCoor.x + m_currentVelocity.x * lookAheadTime, 
            m_handCoor.y + m_currentVelocity.y * lookAheadTime, 
            m_handCoor.z + m_currentVelocity.z * lookAheadTime);
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

    public Vector getHandCoor() {
        return new Vector(m_handCoor);
    }

    public Vector getHandVel() {
        return new Vector(m_handVel);
    }

    // Get angles from x and y input (yay!):

    public static double[] getAnglesFrom2DVector(Vector vector2D) {
        double x = vector2D.x;
        double y = vector2D.y;
        double[] a1a2 = {0, 0};
    
        double DEADBAND = 5;

        boolean negative = false;

        if(x < 0) { negative = true; }

        x = Math.abs(x);

        final double BICEP_LENGTH = 40;
        final double FOREARM_LENGTH = 30;
    
        double a1 = 0;
        double a2 = 0;     
        
        // Jack & Josh's Model:

        double r = Math.sqrt((x * x) + (y * y));

        double theta2Numerator = -(r * r) + (BICEP_LENGTH * BICEP_LENGTH) + (FOREARM_LENGTH * FOREARM_LENGTH);

        double theta2Denominator = 2 * BICEP_LENGTH * FOREARM_LENGTH;

        a2 = Math.toDegrees(Math.acos(theta2Numerator / theta2Denominator));

        double theta1ComplementNumerator = -(FOREARM_LENGTH * FOREARM_LENGTH) + (BICEP_LENGTH * BICEP_LENGTH) + (r * r);

        double theta1ComplementDenominator = 2 * r * BICEP_LENGTH;

        double theta1ComplementUpper = Math.toDegrees(Math.acos(theta1ComplementNumerator / theta1ComplementDenominator));

        double theta1ComplementLower = Math.toDegrees(Math.atan(y / x));

        double theta1Complement = theta1ComplementLower + theta1ComplementUpper;

        a1 = 90 - theta1Complement;

        a1a2[0] = a1;
        a1a2[1] = a2;

        if(negative) {
            a1a2[0] = -a1;
            a1a2[1] = -a2;
        }

        return a1a2;
    }

    public static Vector convert3Dto2D(Vector vector3D) {
        Vector vector2D = new Vector(0,0);
        if (vector2D.is3D()) {
            double x = Vector.mag(new Vector(vector3D.x,vector3D.y));
            double y = vector3D.z;
            vector2D.setValues(x, y);
        }
        return vector2D;
    }

    /**
     * 
     * @param vector3D A target 3D coordinate
     * @return         An array that containes the three angle values necessary
     *                 for the arm to reach the target 3D coordinate, in the order
     *                 {turretAngle, shoulderAngle, elbowAngle}
     */
    public static double[] getAnglesFrom3DVector(Vector vector3D) {
        double[] armAngles = getAnglesFrom2DVector(convert3Dto2D(vector3D));
        double turretAngle = Math.atan(vector3D.y/vector3D.x);
        return new double[] {turretAngle, armAngles[0], armAngles[1]};
    }
}
