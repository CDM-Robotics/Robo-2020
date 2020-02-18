package team6072.robot2020.utility.math;

/**
 * Represents a position on the field. Has a vector to represent the poosition
 * of the robot on the field, and the angle at which the robot is pointing.
 * 
 * The position vector represents the Robot's position on an XY coordinate
 * plane, where X is the axis parrallel to the long edge of the field and Y is
 * the axis parrellel to the driver station walls. Zero on the X axis is the
 * line perpendicular from the center of the driver station wall, while Zero on
 * the Y axis is one of the Driver station wall.
 * 
 * See the README.md at the location below for a pixel art visual of the
 * coordinate plane.
 * 
 * team6072.robot2020.utility.math.README.md
 * 
 */
public class Position2D {

    private Vector2D mVector2D;
    private Angle2D mAngle2D;

    public Position2D() {
        mVector2D = new Vector2D();
        mAngle2D = new Angle2D();
    }

    public Position2D(Vector2D vector2d, Angle2D angle2d) {
        mVector2D = vector2d;
        mAngle2D = angle2d;
    }

    public Position2D(double x, double y, Angle2D angle2d) {
        mVector2D = new Vector2D(x, y);
        mAngle2D = angle2d;
    }

    public Vector2D getPositionVector2D() {
        return mVector2D;
    }

    public Angle2D getAngle2D() {
        return mAngle2D;
    }

    public void setAngle(Angle2D angle2d) {
        mAngle2D = angle2d;
    }

    public void setPosition(Vector2D vector2d) {
        mVector2D = vector2d;
    }

}