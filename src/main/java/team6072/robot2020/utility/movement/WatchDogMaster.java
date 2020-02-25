package team6072.robot2020.utility.movement;

import java.util.ArrayList;

import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.utility.RunAndEndable;

/**
 * Yet to be tested.
 */
public class WatchDogMaster implements RunAndEndable {

    private static WatchDogMaster mWatchDogMaster;
    private LogWrapper mLog;

    private ArrayList<WatchDog> mWatchDogs;
    private boolean mCanRun;

    public static WatchDogMaster getInstance() {
        if (mWatchDogMaster == null) {
            mWatchDogMaster = new WatchDogMaster();
        }
        return mWatchDogMaster;
    }

    /**
     * Sets up all the current WatchDogs
     */
    private WatchDogMaster() {
        mLog = new LogWrapper(FileType.UTILITY, "Watch Dog Master", LoggerConstants.WATCH_DOG_PERMISSION);

        mWatchDogs = new ArrayList<WatchDog>();
        /*
         * create member WatchDog turretWatchDog = new WatchDog(){
         * 
         * @Override public double getCurrentPosition() { Get Turret get current Turret
         * position return curnPosition; } }; add to array list
         * 
         */
    }

    /**
     * STill need to test this
     * 
     * Also need to test if it is worth running the while loops every iteration or
     * just letting the outer loop run
     */
    public void run() {
        mCanRun = true;
        while (mCanRun) {
            for (int i = 0; i < mWatchDogs.size(); i++) {
                if (mWatchDogs.get(i).getCurrentPosition() > mWatchDogs.get(i).getTopBoundaryVal()) {
                    while (mWatchDogs.get(i).getCurrentPosition() > mWatchDogs.get(i).getTopBoundaryVal()) {
                        mWatchDogs.get(i).passedTopBoundaryFunc();
                        try {
                            Thread.sleep(2);
                        } catch (InterruptedException err) {
                            mLog.error(err.toString());
                        }
                    }
                } else if (mWatchDogs.get(i).getCurrentPosition() < mWatchDogs.get(i).getBtmBoundaryVal()) {
                    while (mWatchDogs.get(i).getCurrentPosition() < mWatchDogs.get(i).getBtmBoundaryVal()) {
                        mWatchDogs.get(i).passedBtmBoundaryFunc();
                        try {
                            Thread.sleep(2);
                        } catch (InterruptedException err) {
                            mLog.error(err.toString());
                        }
                    }
                }
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException err) {
                mLog.error(err.toString());
            }
        }
    }

    public void end() {
        mCanRun = false;
    }

    /**
     * This function is the Watchdog, a class that represents the boundaries of a
     * subsystem's mobility.
     * 
     * For example, if I were driving an elevator with a motor, there are boundaries
     * I would want to have installed. First I would want there to be a top
     * boundary, so that the robot does not break the chain limit, and then I would
     * want a bottom boundary so that the robot does not destroy the electronics.
     * Therefore I would make a watchdog that would, 1) sense if the robot passed
     * the top or bottom boundary, and then 2) make the robot drive backwards, at
     * specified values, to get back into the boundaries.
     * 
     * ******************************* NOTE! *************************************
     * You should probably over compensate for mechanical error, if you make the
     * boundary too close to the robot's breaking point, then not even the WatchDog
     * will be fast enough to save it. If you boundary is 10, make the top boundary
     * 8 or 7 to be safe.
     * 
     * 
     * If, in somme case, you would like to make one side have a boundary and the
     * other not have a boundary, just make it a very very very small or large
     * number. Sorry.
     */
    private abstract class WatchDog {

        public double mTopBoundaryVal, mBtmBoundaryVal, mTopBoundaryOutput, mBtmBoundaryOutput;

        /**
         * @param topBoundaryVal    The largest value at which the watchdog will trigger
         * @param btmBoundaryVal    The smallest value at which the watchdog will
         *                          trigger
         * @param topBoundaryOutput The output you want the subsystem to run at when it
         *                          hits the top value
         * @param btmBoundaryOutput The output you want the subsystem to run at when it
         *                          hits to bottom value
         */
        public WatchDog(double topBoundaryVal, double btmBoundaryVal, double topBoundaryOutput,
                double btmBoundaryOutput) {
            mTopBoundaryVal = topBoundaryVal;
            mBtmBoundaryVal = btmBoundaryVal;
            mTopBoundaryOutput = topBoundaryOutput;
            mBtmBoundaryOutput = btmBoundaryOutput;
        }

        public abstract double getCurrentPosition();

        public abstract void passedTopBoundaryFunc();

        public abstract void passedBtmBoundaryFunc();

        public double getTopBoundaryVal() {
            return mTopBoundaryVal;
        }

        public double getBtmBoundaryVal() {
            return mBtmBoundaryVal;
        }

        public double getTopBoundaryOutput() {
            return mTopBoundaryOutput;
        }

        public double getBtmBoundaryOutput() {
            return mBtmBoundaryOutput;
        }
    }

}