package team6072.robot2020.utility.math;


/**
 * Represents a vector.  Could be a position vector, velocity, or acceleration, but it is a vector.
 * 
 * @param x
 * @param y
 */
public class Vector2D {

    private double x, y, mag, magSquared;

    public static Vector2D getVectorFromMagAndDegrees(double mag, double degrees){
        double rad = degrees * 2 * Math.PI / 360;
        double x = mag * Math.cos(rad);
        double y = mag * Math.sin(rad);
        return new Vector2D(x, y);
    }

    public Vector2D(){
        x = 0;
        y = 0;
        mag = 0;
        magSquared = 0;
    }

    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
        this.mag = Math.hypot(x, y);
        this.magSquared = Math.pow(mag, 2);
    }

    public void translateBy(Vector2D vector2d){
        x = x + vector2d.getX();
        y = y + vector2d.getY();
    }


    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    /**
     * Just in case you don't know what Mag is, this function returns the magnitude of the vector usinng the 
     * pathagoreun theorum
     * @return
     */
    public double getMag(){
        return mag;
    }

    /**
     * Returns the vector Magnitude squared
     * @return
     */
    public double getMagSquared(){
        return magSquared;
    }

}