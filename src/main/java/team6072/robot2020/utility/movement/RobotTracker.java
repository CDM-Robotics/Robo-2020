package team6072.robot2020.utility.movement;

import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.subsystems.DriveSys;
import team6072.robot2020.subsystems.NavXSys;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.utility.math.Angle2D;
import team6072.robot2020.utility.math.Position2D;
import team6072.robot2020.utility.math.Vector2D;
import team6072.robot2020.utility.RunAndEndable;

/**
 * The Thread that trasks the robot's position as long as it is running. Has a
 * tolerance level of a few inches. It's not God's eye, what do you expect.
 */
public class RobotTracker implements RunAndEndable {

    private static RobotTracker mRobotTracker;
    private LogWrapper mLog;

    private DriveSys mDriveSys;
    private NavXSys mNavXSys;

    private double mLastLeftMotorPosition;
    private double mLastRightMotorPosition;
    private Vector2D mCurrentPosition;

    private boolean mReady = false;
    private boolean mCanRun = true;

    public static RobotTracker getInstance() {
        if (mRobotTracker == null) {
            mRobotTracker = new RobotTracker();
        }
        return mRobotTracker;
    }

    private RobotTracker() {
        mLog = new LogWrapper(FileType.UTILITY, "Robot Tracker", LoggerConstants.ROBOT_TRACKER_PERMISSION);
        // mLog.print("Starting Robot Tracker");
        // get the subsystems
        mDriveSys = DriveSys.getInstance();
        mNavXSys = NavXSys.getInstance();

        // store the current position on each motor
        // store the current position on the navx
        mLastLeftMotorPosition = mDriveSys.getLeftCurnPosInches();
        mLastRightMotorPosition = mDriveSys.getRightCurnPosInches();
        mLog.debug("Left Motor Start POs", mLastLeftMotorPosition, "Right Motor Start Pos", mLastRightMotorPosition);
    }

    /**
     * This function is where the pseudo-calculus magic happens. THis loop runs 50
     * times a second and so it is perfectly in sync with the Scheduler and also
     * does not overload the Talons with information requests.
     * 
     * 
     * ********************************** NOTE **********************************
     * Some of the concepts utilized below for this function have elements of
     * Calculus in them. If you have not taken Calculus, just wrestle with it for a
     * while, you will get it eventually.
     * 
     * Now for how thiis works. If we the robot is driving forward while curving to
     * the right, it is impossible to know where the robot is going to end up based
     * off only the motor encoders. This is because there are too many combinations
     * of motor outputs that could have resulted in the resulting numbers, and
     * therefore, we can not know where the robot is. However, if the robot is
     * driving in a straight line, at some head angle, we can figure out where it
     * ends, as its angle does not change, and both wheels move the same distance.
     * 
     * If you followed that then this part is the wizardry. Because we cannot find
     * our position when our robot does a curve, but we can find our position when
     * our robot does a straight line, we simply assume our robot is driving in a
     * bunch of tiny straight lines. This works because, in essence, a curved line
     * is a ton of infinitesimally small straight lines put together. So if we
     * simply assume we are driving in a bunch of tiny straight lines, and change
     * the angle of each new line to match the robot, our approximation's error will
     * be basically negligable. And it is work noting as well that running this
     * function 50 times a second, as it currently does, makes our tolerance error
     * about 1 inch on either axis.
     */
    public void run() {
        mLog.print("Thread before loop");
        mCanRun = true;
        while (mCanRun) {
            while (mReady && mCanRun) {
                double deltaDistanceRight = mDriveSys.getRightCurnPosInches() - mLastRightMotorPosition;
                double deltaDistanceLeft = mDriveSys.getLeftCurnPosInches() - mLastLeftMotorPosition;
                double deltaDistance = (deltaDistanceLeft + deltaDistanceRight) / 2;

                Vector2D movementVector = Vector2D.getVectorFromMagAndDegrees(deltaDistance, mNavXSys.getYaw());
                mCurrentPosition.translateBy(movementVector);

                mLastRightMotorPosition = mDriveSys.getRightCurnPosInches();
                mLastLeftMotorPosition = mDriveSys.getLeftCurnPosInches();
                try {
                    Thread.sleep(20);
                } catch (InterruptedException err) {
                    mLog.error(err.toString());
                }
            }
        }
    }

    public void end() {
        mCanRun = false;
    }

    /**
     * Sets the robot's current position vector to the parameter
     * 
     * @param curnPosition
     */
    public void setCurrentPosition(Vector2D curnPosition) {
        mCurrentPosition = curnPosition;
        mReady = true;
    }

    /**
     * Returns position of the robot based on the XY coordinate plane described on
     * the README.md here
     * 
     * team6072.robot2020.utility.math.README.md
     * 
     * @return The robot's current absolute position on the XY coordinate plane
     */
    public Position2D getAbsolutePosition() {
        if (mReady == true) {
            return new Position2D(mCurrentPosition, Angle2D.getAngle2DFromDegrees(mNavXSys.getYaw()));
        } else {
            mLog.error("You haven't initialized the starting position for the Robot Tracker!!! "
                    + "Use RobotTracker.getinstance().setCurrentPosition(vector2D) to fix.");
            return null;
        }
    }

}