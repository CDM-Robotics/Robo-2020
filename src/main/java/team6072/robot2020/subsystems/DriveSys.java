package team6072.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.movement.pid.MyPIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpiutil.math.MathUtil;
import team6072.robot2020.utility.movement.pid.datasources.NavXSource.NavXDataTypes;
import team6072.robot2020.utility.movement.pid.datasources.NavXSource;
import team6072.robot2020.constants.subsystems.DriveSysConstants;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.commands.drivesys.*;
import team6072.robot2020.robot.ControlBoard;
import team6072.robot2020.utility.movement.pathfinder.Pathfinder;
import team6072.robot2020.utility.movement.pathfinder.Pathfinder.Waypoint;
import team6072.robot2020.utility.movement.pathfinder.Pathfinder.Path;

/**
 * This is the drive system of the robot, pretty self explanitory, but I would
 * like to point out one thing real fast Reseting the motors takes time, and
 * therefore, if you reset the motors, you must make sure that nothing takes the
 * position of the robot in the same chunk of code. THis is to prevent the code
 * from taking a reading BEFORE the number gets erased and reset to zero,
 * causing an error in your code, and more specifically ROBOT TRACKER.
 */
public class DriveSys implements Subsystem {

    private LogWrapper mLog;

    private WPI_TalonFX mLeft_Master;
    private WPI_TalonFX mLeft_Slave0;
    private WPI_TalonFX mRight_Master;
    private WPI_TalonFX mRight_Slave0;

    private TalonFXSensorCollection mLeTalonFXSensorCollection;
    private TalonFXSensorCollection mRiTalonFXSensorCollection;

    private static DriveSys mDriveSys;

    public static DriveSys getInstance() {
        if (mDriveSys == null) {
            mDriveSys = new DriveSys();
        }
        return mDriveSys;
    }

    private DriveSys() {
        mLog = new LogWrapper(FileType.SUBSYSTEM, "DriveSystem", LoggerConstants.DRIVESYS_PERMISSION);

        mLeft_Master = new WPI_TalonFX(DriveSysConstants.LEFT_TALON_MASTER);
        mLeft_Slave0 = new WPI_TalonFX(DriveSysConstants.LEFT_TALON_SLAVE0);
        mRight_Master = new WPI_TalonFX(DriveSysConstants.RIGHT_TALON_MASTER);
        mRight_Slave0 = new WPI_TalonFX(DriveSysConstants.RIGHT_TALON_SLAVE0);

        configMotors();

        mLeTalonFXSensorCollection = new TalonFXSensorCollection(mLeft_Master);
        mRiTalonFXSensorCollection = new TalonFXSensorCollection(mRight_Master);

        // mLeTalonFXSensorCollection.setIntegratedSensorPosition(0, 10);
        // mRiTalonFXSensorCollection.setIntegratedSensorPosition(0, 10);
    }

    private void configMotors() {

        mLeft_Slave0.follow(mLeft_Master);
        mRight_Slave0.follow(mRight_Master);

        mLeft_Master.setInverted(DriveSysConstants.DRIVE_LEFT_INVERT);
        mLeft_Slave0.setInverted(InvertType.FollowMaster);
        mRight_Master.setInverted(DriveSysConstants.DRIVE_RIGHT_INVERT);
        mRight_Slave0.setInverted(InvertType.FollowMaster);

        mLeft_Master.setSensorPhase(DriveSysConstants.LEFT_TALON_MASTER_SENSOR_PHASE);
        mLeft_Slave0.setSensorPhase(DriveSysConstants.LEFT_TALON_SLAVE0_SENSOR_PHASE);
        mRight_Master.setSensorPhase(DriveSysConstants.RIGHT_TALON_MASTER_SENSOR_PHASE);
        mRight_Slave0.setSensorPhase(DriveSysConstants.RIGHT_TALON_SLAVE0_SENSOR_PHASE);

        mLeft_Master.configOpenloopRamp(DriveSysConstants.DRIVE_CONFIG_OPEN_LOOP_RAMP,
                DriveSysConstants.DRIVE_TIME_OUT);
        mRight_Master.configOpenloopRamp(DriveSysConstants.DRIVE_CONFIG_OPEN_LOOP_RAMP,
                DriveSysConstants.DRIVE_TIME_OUT);

        mLeft_Master.setNeutralMode(DriveSysConstants.DRIVE_NEUTRAL_MODE);
        mRight_Master.setNeutralMode(DriveSysConstants.DRIVE_NEUTRAL_MODE);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ArcadeDriveCmd(ControlBoard.getInstance().mDriveStick));
    }

    /**
     * 
     * For Testing the direction.
     * 
     */
    public void testMotor() {
        mLeft_Master.set(ControlMode.PercentOutput, .7);
        mRight_Master.set(ControlMode.PercentOutput, .7);
    }

    /***********************************************************
     * 
     *
     * Arcade Drive stuff
     * 
     * 
     ***********************************************************/
    /**
     * This Arcade drive function is one that I have written myself as to make the
     * process more efficient and customizable, while still not messing with
     * PurePursuit and Watchdog code.
     * 
     * Let me explain some things. This function is complicated because it's purpose
     * is to make sure the wheels do not go faster than the user would want.
     * 
     * The way we do this is by first deciding what the maximum speed is, labeled
     * maxInput. Then, when we decide how fast we want the fastest motor to go, we
     * then have to decide which motor will get to have that maxInput value. After
     * using some If statements, we find out which motor must go faster to be able
     * to turn the way we want. We set that motor to the max speed and then do
     * addition to find the second motor's speed. The math done to find the second
     * motor's speed will vary depending on the mag and yaw values.
     * 
     * If everything is right then the function should be completely normal
     * 
     * @param mag
     * @param yaw
     */

    public void arcadeDrive(double mag, double yaw) {
        // clamp numbers
        // makes the numbers within the range of 1 and -1
        mag = MathUtil.clamp(mag, -1.0, 1.0);
        yaw = MathUtil.clamp(yaw, -1.0, 1.0);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        // This makes the input graph to power look like a parabola and it makes the
        // user able to make finer turns and controls closer to the joystick, while
        // still allowing the user to reach the maximum speed
        mag = Math.copySign(mag * mag, mag);
        yaw = Math.copySign(yaw * yaw, yaw);

        double leftOutput;
        double rightOutput;

        // the speed the faster motor will be going
        double maxInput = Math.copySign(Math.max(Math.abs(mag), Math.abs(yaw)), mag); 

        // If the user wants to make the robot turn in place, by making the yaw greater
        // than the mag, then reduce the motor's fastest speed by 10%
        if (Math.abs(yaw) > Math.abs(mag)) {
            maxInput = maxInput * 0.9;
        }

        // Deciding which motor will be the faster one and which will be the slower one
        if (mag >= 0.0) {
            // First quadrant, else second quadrant
            if (yaw >= 0.0) {
                leftOutput = maxInput;
                rightOutput = mag - yaw;
            } else {
                leftOutput = mag + yaw;
                rightOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (yaw >= 0.0) {
                leftOutput = mag + yaw;
                rightOutput = maxInput;
            } else {
                leftOutput = maxInput;
                rightOutput = mag - yaw;
            }
        }

        // setting values
        mLeft_Master.set(MathUtil.clamp(leftOutput, -1.0, 1.0));
        mRight_Master.set(MathUtil.clamp(rightOutput, -1.0, 1.0));

        mLog.periodicPrint("Mag: " + mag + " yaw: " + yaw, 20);
    }

    /***********************************************************
     * 
     *
     * Relative Drive stuff
     * 
     * 
     ***********************************************************/

    private double RELATIVE_P = DriveSysConstants.RELATIVE_P;
    private double RELATIVE_D = DriveSysConstants.RELATIVE_D;
    private final double RELATIVE_I = DriveSysConstants.RELATIVE_I;
    private final double RELATIVE_F = DriveSysConstants.RELATIVE_F;

    private final double RELATIVE_UPPER_DEADBAND = DriveSysConstants.RELATIVE_UPPER_DEADBAND;
    private final double RELATIVE_LOWER_DEADBAND = DriveSysConstants.RELATIVE_LOWER_DEADBAND;
    private final double BASE_PERCENT_OUT = DriveSysConstants.BASE_PERCENT_OUT;

    private final double RELATIVE_YAW_THRESHOLD = DriveSysConstants.RELATIVE_YAW_TOLERANCE;

    private MyPIDController mRelativePIDController;
    private NavXSource mNavXSource;

    /**
     * This function sets up the PID constants of the RelativePIDController and the
     * NavXsource to relay the information to the PID - called in the
     * RelativeDriveCmd.initialize() function
     */
    public void initRelativeDrive() {
        mNavXSource = new NavXSource(NavXDataTypes.TOTAL_YAW);
        mRelativePIDController = new MyPIDController(RELATIVE_P, RELATIVE_I, RELATIVE_D, RELATIVE_F, mNavXSource, 1,
                -1);
        mRelativePIDController.start();
        mLog.warning("REMEMBER TO SET THE DEADBAND ON THE RELATIVE DRIVE SYSTEM!!!!!");
        // mRelativePIDController.setDeadband(RELATIVE_UPPER_DEADBAND,
        // RELATIVE_LOWER_DEADBAND, BASE_PERCENT_OUT);
    }

    /**
     * This function takes the target angle and sets the PID to that target. It then
     * uses Robot.arcadeDrive to drive the robot to that yaw position. Then, under a
     * certian tollerance, when the yaw is close enough to the target yaw angle, the
     * robot drives forward with the specified magnitude.
     * 
     * @param targetAngle
     * @param magnitude
     */
    public void executeRelativeDrive(double targetAngle, double magnitude) {
        mRelativePIDController.setSetpoint(targetAngle);
        double yaw = mRelativePIDController.getOutput();
        mLog.periodicDebug(30, "Yaw", yaw);
        if (yaw > RELATIVE_YAW_THRESHOLD) {
            arcadeDrive(0.0, yaw);
        } else {
            arcadeDrive(magnitude, yaw);
        }
    }

    /*------------------------------------------------------
    *
    *
    *Pure Pursuit Things 
    *
    *
    -------------------------------------------------------*/
    Path m_robotPath;

    public void initPurePursuit() {

    }

    private double m_leftVelocityCalc;
    private double m_rightVelocityCalc;

    public void executePurePursuit(int i) {

        if (i == 1) {

            Waypoint[] waypoints = new Waypoint[3];

            waypoints[0] = new Waypoint(0, 0, 90);
            waypoints[1] = new Waypoint(0, 5, 90);
            waypoints[2] = new Waypoint(0, 10, 90);

            Path AutoPath = Pathfinder.computePath(waypoints, 500, 0.02, 1, 1, 0.3, 2.6);

            m_robotPath = AutoPath;

        }

        if (i < 501) {

            m_leftVelocityCalc = m_robotPath.m_rightPath[i].velocity / 2;
            m_rightVelocityCalc = m_robotPath.m_leftPath[i].velocity / 2;

            mRight_Master.set(m_rightVelocityCalc);

            System.out.println(m_rightVelocityCalc);

            mLeft_Master.set(-1 * m_leftVelocityCalc);

            System.out.println(m_leftVelocityCalc);

        } else if (i > 501) {
            mLeft_Master.set(ControlMode.PercentOutput, 0);
            mRight_Master.set(ControlMode.PercentOutput, 0);
        }

    }

    /**
     * 
     * 
     * 
     * Get Functions
     * 
     * 
     * 
     */

    // POsition //

    /**
     * Gets the current position of the Left motor in Ticks This number comes out as
     * negative on the robot so I added a negative sign so that it matches the Right
     * side.
     * 
     * @return
     */
    public double getLeftCurnPosTicks() {
        return -mLeTalonFXSensorCollection.getIntegratedSensorPosition();
    }

    /**
     * Gets the current position of the Right motor in Ticks
     * 
     * @return
     */
    public double getRightCurnPosTicks() {
        return mRiTalonFXSensorCollection.getIntegratedSensorPosition();
    }

    /**
     * Gets the current position of the Left motor in Inches
     * 
     * @return
     */
    public double getLeftCurnPosInches() {
        double position = getLeftCurnPosTicks();
        double inches = ((position / 2048d) / 5d) * 6d * Math.PI;
        return inches;
    }

    /**
     * Gets the current position of the Right motor in Inches
     * 
     * @return
     */
    public double getRightCurnPosInches() {
        double position = getRightCurnPosTicks();
        double inches = ((position / 2048d) / 5d) * 6d * Math.PI;
        return inches;
    }

    // Velocity //

    /**
     * Units that come in are the number of ticks that pass in 100ms. We convert
     * this to ticks per second by dividing that number by 100ms and then
     * multipllying it by 1000ms (1 sec). Therefore making the returning units the
     * number of ticks that pass per second
     * 
     * @return Ticks passed per second
     */
    public double getLeftCurnVelTicks() {
        return (mLeTalonFXSensorCollection.getIntegratedSensorVelocity() / 100d) * 1000d;
    }

    /**
     * Units that come in are the number of ticks that pass in 100ms. We convert
     * this to ticks per second by dividing that number by 100ms and then
     * multipllying it by 1000ms (1 sec). Therefore making the returning units the
     * number of ticks that pass per second
     * 
     * @return Ticks passed per second
     */
    public double getRightCurnVelTicks() {
        return (mRiTalonFXSensorCollection.getIntegratedSensorVelocity() / 100d) * 1000d;
    }

    /**
     * Units are Inches per second
     * 
     * @return
     */
    public double getLeftCurnVelInches() {
        double position = getLeftCurnVelTicks();
        double inches = ((position / 2048d) / 5d) * 6d * Math.PI;
        return inches;
    }

    /**
     * Units are inches per second
     * 
     * @return
     */
    public double getRightCurnVelInches() {
        double position = getRightCurnVelTicks();
        double inches = ((position / 2048d) / 5d) * 6d * Math.PI;
        return inches;
    }

}