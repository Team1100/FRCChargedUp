package frc.robot.helpers;
import java.awt.geom.Point2D;
import frc.robot.helpers.Point3D;
import frc.robot.subsystems.Arm;


public class ArmSegmentHelper {

    // TODO: replace with accurate values
    public static final double SHOULDER_LENGTH = 2; // in feet
    public static final double FOREARM_LENGTH = 1.4; // in feet
    public static final Point3D SHOULDER_JOINT_COOR = new Point3D(0,0,1);
    // Used for the overextention prevention
    // TODO: replace with accurate values
    public static final double ROBOT_WIDTH = 2;
    public static final double ROBOT_LENGTH = 3;
    
    private Point3D m_handCoor;

    Arm m_arm;
    
    public ArmSegmentHelper() {
        m_arm = Arm.getInstance();
        // TODO: Initially has m_handCoor set as if the arm is straight up with the forearm folded down, may want to initialize with a more accurate value.
        m_handCoor = new Point3D(0,0, SHOULDER_LENGTH - FOREARM_LENGTH + SHOULDER_JOINT_COOR.getZ());
    }

    public Point3D calculateHandCoor() {
        double x = SHOULDER_JOINT_COOR.getX() + Math.cos(Math.toRadians(m_arm.getTurretAngle())) *
            (SHOULDER_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle())));
            

        double y = SHOULDER_JOINT_COOR.getY() + Math.sin(Math.toRadians(m_arm.getTurretAngle())) *
            (SHOULDER_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle())));


        double z = SHOULDER_JOINT_COOR.getZ() 
            + SHOULDER_LENGTH * Math.sin(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.sin(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle()));

        m_handCoor = new Point3D(x,y,z);
        return m_handCoor;
        
    }

    // TODO: Create checkForOverextention() method
}
