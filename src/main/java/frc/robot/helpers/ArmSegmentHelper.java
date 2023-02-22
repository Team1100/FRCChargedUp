import java.awt.geom.Point3D;

public class ArmSegmentHelper {

    public static final int SHOULDER_LENGTH = 2; // in feet
    public static final int FOREARM_LENGTH = 1.4 // in feet
    public static final Point3D.Double SHOULDER_JOINT_COOR = new Point2D.Double(0,0,0);
    
    private Point3D.Double m_handCoor;

    Arm m_arm;
    
    public ArmSegmentHelper() {
        m_arm = Arm.getInstance();
        // TODO: Initially has m_handCoor set at (0,0,0), may want to initialize with a more accurate value.
        m_handCoor = new Point3D.Double(0,0,0);
    }

    public double calculateHandCoor() {
        double x = SHOULDER_JOINT_COOR.getX() + Math.cos(Math.toRadians(m_turretAngle)) *
            (SHOULDER_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle())));
            

        double y = SHOULDER_JOINT_COOR.getY() + Math.sin(Math.toRadians(m_turretAngle)) *
            (SHOULDER_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.cos(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle())));
            

        double z = SHOULDER_JOINT_COOR.getZ() 
            + SHOULDER_LENGTH * Math.sin(Math.toRadians(m_arm.getShoulderAngle())) 
            + FOREARM_LENGTH * Math.sin(Math.toRadians(m_arm.getShoulderAngle() + m_arm.getElbowAngle()));

        m_handCoor.setLocation(x,y,z);
    }

    // TODO: Create checkForOverextention() method
}
