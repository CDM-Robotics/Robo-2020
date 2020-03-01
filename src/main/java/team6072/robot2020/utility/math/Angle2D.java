package team6072.robot2020.utility.math;


/**
 * Represents an angle
 */
public class Angle2D {
    private final double m_value;
    private final double m_cos;
    private final double m_sin;

    /**
     * Constructs a Angle2D with a default angle of 0 degrees.
     */
    public Angle2D() {
        m_value = 0.0;
        m_cos = 1.0;
        m_sin = 0.0;
    }

    /**
     * Constructs a Angle2D with the given radian value. The x and y don't have
     * to be normalized.
     *
     * @param degrees The value of the angle in radians.
     */
    public Angle2D(double degrees) {        
        m_value = Math.toRadians(degrees);
        m_cos = Math.cos(m_value);
        m_sin = Math.sin(m_value);
    }

    /**
     * Constructs a Angle2D with the given x and y (cosine and sine) components.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sine of the rotation.
     */
    @SuppressWarnings("ParameterName")
    public Angle2D(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-6) {
            m_sin = y / magnitude;
            m_cos = x / magnitude;
        } else {
            m_sin = 0.0;
            m_cos = 1.0;
        }
        m_value = Math.atan2(m_sin, m_cos);
    }

    /**
     * Constructs and returns a Angle2D with the given degree value.
     *
     * @param degrees The value of the angle in degrees.
     * @return The rotation object with the desired angle value.
     */
    public static Angle2D fromDegrees(double degrees) {
        return new Angle2D(Math.toRadians(degrees));
    }

    /**
     * Constructs and returns a Angle2D with the given radian value.
     *
     * @param radians The value of the angle in radians.
     * @return The rotation object with the desired angle value.
     */
    public static Angle2D fromRadians(double radians) {
        return new Angle2D(radians);
    }

    /*
     * Returns the radian value of the rotation.
     *
     * @return The radian value of the rotation.
     */
    public double getRadians() {
        return m_value;
    }

    /**
     * Returns the degree value of the rotation.
     *
     * @return The degree value of the rotation.
     */
    public double getDegrees() {
        return Math.toDegrees(m_value);
    }

    /**
     * Returns the cosine of the rotation.
     *
     * @return The cosine of the rotation.
     */
    public double getCos() {
        return m_cos;
    }

    /**
     * Returns the sine of the rotation.
     *
     * @return The sine of the rotation.
     */
    public double getSin() {
        return m_sin;
    }

    /**
     * Returns the tangent of the rotation.
     *
     * @return The tangent of the rotation.
     */
    public double getTan() {
        return m_sin / m_cos;
    }

    @Override
    public String toString() {
        return String.format("Angle2D(Rads: %.2f, Deg: %.2f)", m_value, Math.toDegrees(m_value));
    }

    /**
     * Checks equality between this Angle2D and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Angle2D) {
            return Math.abs(((Angle2D) obj).m_value - m_value) < 1E-9;
        }
        return false;
    }

}
