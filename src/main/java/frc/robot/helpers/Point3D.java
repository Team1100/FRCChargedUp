package frc.robot.helpers;
public class Point3D {
    private double x;
    private double y;
    private double z;
    
    public Point3D(double x0, double y0, double z0) {
       x = x0;
       y = y0;
       z = z0;
    }
    
    public double getX() {
        return x;
    }
    
    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }
    

    public void setLocation(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

}