package team6072.robot2020.utility.math;


public class Position2D{

    private Vector2D mVector2D;
    private Angle2D mAngle2D;

    public Position2D(){
        mVector2D = new Vector2D();
        mAngle2D = new Angle2D();
    }

    public Position2D(Vector2D vector2d, Angle2D angle2d){
        mVector2D = vector2d;
        mAngle2D = angle2d;
    }

    public Position2D(double x, double y, Angle2D angle2d){
        mVector2D = new Vector2D(x, y);
        mAngle2D = angle2d;
    }

    public Vector2D getPositionVector2D(){
        return mVector2D;
    }

    public Angle2D getAngle2D(){
        return mAngle2D;
    }

    public void setAngle(Angle2D angle2d){
        mAngle2D = angle2d;
    }

    public void setPosition(Vector2D vector2d){
        mVector2D = vector2d;
    }


}