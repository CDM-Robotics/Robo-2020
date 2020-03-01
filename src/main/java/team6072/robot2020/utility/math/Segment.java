package team6072.robot2020.utility.math;

import team6072.robot2020.utility.math.Vector2D;

public class Segment {

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

    /**
     * Gets the distance from the Robot's position to the closest point on the
     * segment
     * 
     * This function essentially draws a perpendicular line from the segment to the
     * robot's position and tells us the distance(length) of the line drawn
     * 
     * Uses some pretty funky vector maths :)
     */
    public double getPerpendicularDistance(Vector2D point) {
        Vector2D seg = getDelta();
        Vector2D vectorToPoint = getStart().inverse().translateBy(point); // the robot's position relative to the
                                                                           // starting point of the segment
        Vector2D scaledDownSeg = seg.getProjectionVector(vectorToPoint);
        Vector2D displacementVector = scaledDownSeg.inverse().translateBy(vectorToPoint);
        double displacementFromSegment = displacementVector.getMag();
        return displacementFromSegment;
    }

    /**
     * Important!
     */
    public double getParrellelDistance(Vector2D point){
        Vector2D seg = getDelta();
        Vector2D vectorToPoint = getStart().inverse().translateBy(point); // the robot's position relative to the
                                                                           // starting point of the segment
        double dotProduct = vectorToPoint.dotProduct(seg);
        double parrellelDistance = dotProduct / seg.getMag();
        return parrellelDistance;
    }

}