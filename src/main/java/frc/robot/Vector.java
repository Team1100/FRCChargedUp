// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Vector {
    public double x;
    public double y;
    public double z;

    private static boolean is3D;

    public Vector() {
        x = 0;
        y = 0;
        z = 0;
        is3D = true;
    }

    public Vector(double X, double Y) {
        x = X;
        y = Y;
        is3D = false;
    }

    public Vector(double X, double Y, double Z) {
        x = X;
        y = Y;
        z = Z;
        is3D = true;
    }

    public Vector(Vector a) {
        x = a.x;
        y = a.y;
        if (a.is3D()) {
            z = a.z;
            is3D = true;
        }

    }

    public static double dot(Vector a, Vector b) {
        double result = a.x * b.x + a.y * b.y;
        if (is3D) {
            result += + a.z * b.z;
        } 
        return result;
    }

    // Method to project vector A onto vector B
    public static Vector proj(Vector a, Vector b) {
        Vector result = new Vector(b.x, b.y);
        double a_dot_b = dot(a, b);
        double mag_b = mag(b);
        multScalar(result, (a_dot_b) / (mag_b * mag_b));
        return result;
    }

    public static Vector unit(Vector a) {
        Vector result;
        if (is3D) {
            result = new Vector(a.x,a.y,a.z);
        } else {
            result = new Vector(a.x,a.y);
        }

        double mag_a = mag(a);
        divScalar(result,mag_a);
        
        return result;
    }

    public static void addScalar(Vector a, double s) {
        a.x += s;
        a.y += s;
        if (is3D) {
            a.z += s;
        }
    }

    public static void multScalar(Vector a, double s) {
        a.x *= s;
        a.y *= s;
        if (is3D) {
            a.z *= s;
        }
    }

    public static void divScalar(Vector a, double s) {
        a.x /= s;
        a.y /= s;
        if (is3D) {
            a.z /= s;
        }
    }

    public static double mag(Vector a) {
        double result = Math.sqrt(a.x * a.x + a.y * a.y);
        if (is3D) {
            result =  Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        }
        return result;
    }

    public void setVectorType(boolean is3D) {
        this.is3D = is3D;
    }

    public boolean is3D() {
        return is3D;
    }

    public void setValues(Vector a) {
        this.x = a.x;
        this.y = a.y;
        this.z = a.z;
    }

    public void setValues(double X, double Y, double Z) {
        this.x = X;
        this.y = Y;
        this.z = Z;
    }

    public void setValues(double X, double Y) {
        this.x = X;
        this.y = Y;
    }
}
