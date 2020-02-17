package team6072.robot2020.utility.math;

public class Angle2D {

    private double radians, degrees, cos, sin;

    public static Angle2D getAngle2DFromRadians(double rad) {
        return new Angle2D(rad);
    }

    public static Angle2D getAngle2DFromDegrees(double degrees) {
        double rad = degrees * 2 * Math.PI / 360;
        return new Angle2D(rad);
    }

    public static Angle2D getAngle2DFromSinCos(double sin, double cos) {
        return new Angle2D(sin, cos);
    }

    public Angle2D() {
        radians = 0;
        degrees = 0;
        cos = 1;
        sin = 0;
    }

    public Angle2D(double radians) {
        this.radians = radians;
        this.degrees = radians / (2 * Math.PI) * 360;
        this.cos = Math.cos(this.radians);
        this.sin = Math.sin(this.radians);
    }

    public Angle2D(double sin, double cos) {
        this.cos = cos;
        this.sin = sin;
        double cosRad = Math.acos(cos); // 0 to PI
        double sinRad = Math.asin(sin);
        if (sinRad < 0) {
            cosRad = (2 * Math.PI) - cosRad;
        }
        this.radians = cosRad;
        this.degrees = radians / (2 * Math.PI) * 360;
    }

    public double getRadians(){
        return radians;
    }

    public double getDegrees(){
        return degrees;
    }

    public double getCos(){
        return cos;
    }

    public double getSin(){
        return sin;
    }

}