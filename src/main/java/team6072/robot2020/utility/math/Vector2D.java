package team6072.robot2020.utility.math;

/**
 * Represents a vector. Could be a position vector, velocity, or acceleration,
 * but it is a vector.
 * 
 * @param x
 * @param y
 */
public class Vector2D {

    private double x, y, mag, magSquared;

    public static Vector2D addVectors(Vector2D vector2d, Vector2D vector2d2) {
        vector2d2.translateBy(vector2d);
        return vector2d2;
    }

    public static double dotProduct(Vector2D vector2d, Vector2D vector2d2){
        return (vector2d.getX() * vector2d2.getX() + vector2d.getY() * vector2d2.getY());
    }

    public static Vector2D scaleVector(Vector2D vector2d, double scalar){
        return new Vector2D((vector2d.getX() * scalar), (vector2d.getY() * scalar));
    }

    /**
     * This function creates a vector by using magnitude and degrees rather than an
     * x and y coordinate. This is for convienence rather than necessity.
     * 
     * @param magnitude Distance or magnitude of the vector
     * @param degrees   Degrees from zero. in Degrees
     * @return Vector2D that represents the magnitude and direction specified by the
     *         params
     */
    public static Vector2D getVectorFromMagAndDegrees(double magnitude, double degrees) {
        double rad = degrees * 2 * Math.PI / 360;
        double x = magnitude * Math.cos(rad);
        double y = magnitude * Math.sin(rad);
        return new Vector2D(x, y);
    }

    /**
     * This function creates a vector by using magnitude and degrees rather than an
     * x and y coordinate. This is for convienence rather than necessity.
     * 
     * @param magnitude Distance or magnitude of the vector
     * @param degrees   Angle from zero. In Radians
     * @return Vector2D that represents the magnitude and direction specified by the
     *         params
     */
    public static Vector2D getVectorFromMagAndRadians(double magnitude, double radians) {
        double x = magnitude * Math.cos(radians);
        double y = magnitude * Math.sin(radians);
        return new Vector2D(x, y);
    }

    /**
     * @param x The x component of the vector
     * @param y The y component of the vector
     * @return a Vector2D that represents the x and y commponent vectors specified
     *         by the x and y params
     */
    public static Vector2D getVectorFromXAndY(double x, double y) {
        return new Vector2D(x, y);
    }

    public Vector2D() {
        x = 0;
        y = 0;
        mag = 0;
        magSquared = 0;
    }

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
        this.mag = Math.hypot(x, y);
        this.magSquared = Math.pow(mag, 2);
    }

    public void translateBy(Vector2D vector2d) {
        x = x + vector2d.getX();
        y = y + vector2d.getY();
    }

    /**
     * Applies a rotation to the translation in 2d space.
     *
     * This uses Linear Algebra to function... YEAHhhhhhh I'm not gonna explain it.
     * Ask me if you REALLy need to know
     * 
     * This multiplies the translation vector by a counterclockwise rotation matrix
     * of the given angle. [x_new] [other.cos, -other.sin][x] [y_new] = [other.sin,
     * other.cos][y]
     *
     * For example, rotating a Translation2d of {2, 0} by 90 degrees will return a
     * Translation2d of {0, 2}.
     *
     * @param angle The rotation to rotate the translation by.
     * @return The new rotated translation.
     */
    public Vector2D rotateBy(Angle2D angle) {
        return new Vector2D(x * angle.getCos() - y * angle.getSin(), x * angle.getSin() + y * angle.getCos());
    }

    /**
     * get x component of vector
     * 
     * @return
     */
    public double getX() {
        return x;
    }

    /**
     * get y component of vector
     * 
     * @return
     */
    public double getY() {
        return y;
    }

    /**
     * Just in case you don't know what Mag is, this function returns the magnitude
     * of the vector usinng the pathagoreun theorum
     * 
     * @return
     */
    public double getMag() {
        return mag;
    }

    /**
     * Returns the vector Magnitude squared
     * 
     * @return
     */
    public double getMagSquared() {
        return magSquared;
    }

    public Vector2D inverse() {
        return new Vector2D(-x, -y);
    }

    public Angle2D getAngle(){
        double cos = x / getMag();
        double sin = y / getMag();
        return new Angle2D(sin, cos);
    }

}