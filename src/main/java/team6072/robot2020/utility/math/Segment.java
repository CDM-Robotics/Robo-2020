package team6072.robot2020.utility.math;

import team6072.robot2020.utility.math.Vector2D;
public class Segment{

    private Vector2D start, end, delta;
    private double deltaDist, deltaDistSquared;

    public Segment(double xStart, double yStart, double xEnd, double yEnd) {
        this(new Vector2D(xStart, yStart), new Vector2D(xEnd, yEnd));
    }

    public Segment(Vector2D start, Vector2D end) {
        this.start = start;
        this.end = end;
        delta = start.inverse().translateBy(end);
        deltaDist = Math.hypot(delta.getX(), delta.getY());
        deltaDistSquared = Math.pow(deltaDist, 2);
    }

    public Vector2D getStart() {
        return start;
    }

    public Vector2D getEnd() {
        return end;
    }

    /**
     *
     * @return X and Y offset from the start to the end of the segment.
     */
    public Vector2D getDelta() {
        return delta;
    }

    /**
     *
     * @return Total distance from start of the segment to the end.
     */
    public double getDistance() {
        return deltaDist;
    }

    public double getDistanceSquared() {
        return deltaDistSquared;
    }

    
}