package team6072.robot2020.watchdog;

import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.Timer;

/**
 * This doesn't work yet either...
 */
public class Watchdog {

    private Timer mWatchDog;
    private TimerTask mWatchDogTask;
    private boolean mMoveable;

    /**
     * Watchdog sets up the timer and timser task for you - Note that the thing
     * being tracked are the Quadtrature encoder ticks
     * 
     * - This function is different in that it does not set the Talon to its
     * defaultOutput
     * 
     * @param limit The number you want the motor to stop at
     * @param talon The Talon being affected
     */
    public Watchdog(int limit, TalonSRX talon) {
        mMoveable = true;
        mWatchDog = new Timer();
        mWatchDogTask = new TimerTask() {

            @Override
            public void run() {
                int quad = talon.getSensorCollection().getQuadraturePosition();
                if (quad > limit) {
                    mMoveable = false;
                } else {
                    mMoveable = true;
                }
            }
        };
        mWatchDog.schedule(mWatchDogTask, 1000, 50);

    }

    /**
     * Watchdog sets up the timer and timser task for you - Note that the thing
     * being tracked are the Quadtrature encoder ticks
     * 
     * -This function is different in that it re-enables the motor immediately upon
     * reentering the sensor boundary
     * 
     * @param limit         The number you want the motor to stop at
     * @param defaultOutput The number the motor will become when the limit is
     *                      reached
     * @param talon         The Talon being affected
     */
    public Watchdog(int limit, int defaultOutput, TalonSRX talon) {
        mMoveable = true;
        mWatchDog = new Timer();
        mWatchDogTask = new TimerTask() {

            @Override
            public void run() {
                int quad = talon.getSensorCollection().getQuadraturePosition();
                if (quad > limit) {
                    talon.set(ControlMode.PercentOutput, defaultOutput);
                    mMoveable = false;
                } else {
                    mMoveable = true;
                }
            }
        };
        mWatchDog.schedule(mWatchDogTask, 1000, 50);

    }

    /**
     * Watchdog sets up the timer and timser task for you - Note that the thing
     * being tracked are the Quadtrature encoder ticks
     * 
     * - This function is different in that it will not re-enable the motor until
     * the sensor moves back within a specified enable limit
     * 
     * @param limit         The number you want the motor to stop at
     * @param defaultOutput The number the motor will become when the limit is
     *                      reached
     * @param talon         The Talon being affected
     * @param enableLimit   The number that will turn the motor back on
     */
    public Watchdog(int limit, int defaultOutput, TalonSRX talon, int enableLimit) {
        mMoveable = true;
        mWatchDog = new Timer();
        mWatchDogTask = new TimerTask() {

            @Override
            public void run() {
                int quad = talon.getSensorCollection().getQuadraturePosition();
                if (quad > limit) {
                    talon.set(ControlMode.PercentOutput, defaultOutput);
                    mMoveable = false;
                } else if (quad < enableLimit) {
                    mMoveable = true;
                }

            }
        };
        mWatchDog.schedule(mWatchDogTask, 1000, 50);

    }
    
    /**
     * Returns if the motor is allowed to move or not To use this, you may - 1 set
     * this in front of each move function to properly kill the motor if it goes
     * outside the limit - 2 or set this in front of a repeating function to kill
     * the motor
     */
    public boolean canMove() {
        return mMoveable;
    }

}