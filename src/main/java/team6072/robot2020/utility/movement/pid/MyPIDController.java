package team6072.robot2020.utility.movement.pid;

import team6072.robot2020.constants.pid.PIDControllerConstants;
import team6072.robot2020.utility.movement.pid.datasources.DataSourceBase;
import java.util.ArrayList;

import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.Permission;
import team6072.robot2020.utility.Threaded;
import team6072.robot2020.utility.logging.LogWrapper.FileType;

public class MyPIDController extends Threaded{

    private LogWrapper mLog;

    // Blocker booleans // to prevent things from going too early or too long
    private boolean hasDeadBand = false;

    // constants //
    private final int TIME_INBETWEEN_EXECUTIONS = PIDControllerConstants.TIME_INBETWEEN_EXECUTIONS;
    
    private double mP = 0.0;
    private double mI = 0.0;
    private double mD = 0.0;
    private double mF = 0.0;
    private double mUpperDeadband;
    private double mLowerDeadband;
    private double mBasePercentOut;
    private double mMaxOutput;
    private double mMinOutput;
    private double mSetpoint;

    private double mPriorPosition;
    private double mAccumulatedError = 0;
    private double mOutput = 0;

    private DataSourceBase mDataSource;

    private boolean mRunnable = true;

    /**
     * Remember to use PIDCalc.start() to start the thread and PIDCalc.end() to
     * stop the thread
     * Other than that it is a normal PIDcontroller function, with the addition of Deadbanding( which still doesn't work...)
     * 
     * @param p
     * @param i
     * @param d
     * @param f
     * @param deadband
     * @param dataSource
     * @param maxOutput
     * @param minOutput
     */
    public MyPIDController(double p, double i, double d, double f, DataSourceBase dataSource, double maxOutput,
            double minOutput) {
        mLog = new LogWrapper(FileType.UTILITY, "PID Controller", LoggerConstants.PID_CONTROLLER_PERMISSION);
        mDataSource = dataSource;
        mP = (p);
        mLog.print("mP: " + mP);
        mI = (i);
        mD = (d);
        mF = (f);
        mMaxOutput = maxOutput;
        mMinOutput = minOutput;
        mPriorPosition = dataSource.getData();
        mSetpoint = 0.0;
    }

    public void end(){
        mRunnable = false;
    }

    public double getOutput() {
        return mOutput;
    }

    public void setSetpoint(double setpoint) {
        mSetpoint = setpoint;
        mAccumulatedError = 0;
    }

    public boolean polarity(double err) {
        if (err > 0) {
            return true;
        } else {
            return false;
        }
    }

    public void run() {
        while(mRunnable){

            double curnPosition = mDataSource.getData();
            double err = mSetpoint - curnPosition;
            double rateOfChange = (mPriorPosition - curnPosition) / TIME_INBETWEEN_EXECUTIONS;
    
            double output = (err * mP) + (mAccumulatedError * mI) + -(rateOfChange * mD) + mF;
    
            if (output > mMaxOutput) {
                output = mMaxOutput;
            }
            if (output < mMinOutput) {
                output = mMinOutput;
            }
            if (hasDeadBand) {
                if (output > mBasePercentOut) {
                    double targetPercentOut = (output - mBasePercentOut) / (mMaxOutput - mBasePercentOut);
                    double scaledOutput = (targetPercentOut * (mMaxOutput - mUpperDeadband)) + mUpperDeadband;
                    output = scaledOutput;
                } else if (output < mBasePercentOut) {
                    double targetPercentOut = (output - mBasePercentOut) / (mMinOutput - mBasePercentOut);
                    double scaledOutput = (targetPercentOut * (mMinOutput - mLowerDeadband)) - mLowerDeadband;
                    output = scaledOutput;
                }
            }
            mOutput = output;
            mPriorPosition = curnPosition;
            mAccumulatedError = mAccumulatedError + err * TIME_INBETWEEN_EXECUTIONS;
            mLog.periodicPrint("curnPosition: " + curnPosition + ", setPoint: " + mSetpoint + ", Output: " + mOutput, 30);
            try {
                Thread.sleep(TIME_INBETWEEN_EXECUTIONS);
            } catch (InterruptedException e) {
                System.out.println(e);
            }
        }
    }

    public void setDeadband(double upperDeadband, double lowerDeadband, double basePercentOut) {
        hasDeadBand = true;
        mUpperDeadband = upperDeadband;
        mLowerDeadband = lowerDeadband;
        mBasePercentOut = basePercentOut;

    }

}