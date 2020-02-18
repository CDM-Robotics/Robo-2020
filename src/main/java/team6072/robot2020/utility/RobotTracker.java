package team6072.robot2020.utility;

import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.subsystems.DriveSys;
import team6072.robot2020.subsystems.NavXSys;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.utility.math.Angle2D;
import team6072.robot2020.utility.math.Position2D;
import team6072.robot2020.utility.math.Vector2D;
import team6072.robot2020.utility.thread.RunAndEndable;

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
    }

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
            }
            mLog.warning("You still haven't initialized the starting position!");
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