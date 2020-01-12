package team6072.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.pid.MyPIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import team6072.robot2020.datasources.NavXSource;
import team6072.robot2020.datasources.NavXSource.NavXDataTypes;
import team6072.robot2020.constants.subsystems.DriveSysConstants;
import team6072.robot2020.logging.LogWrapper;
import team6072.robot2020.logging.LogWrapper.FileType;

public class DriveSys implements Subsystem {

    private static DriveSys mDriveSys;
    private LogWrapper mLog;

    private WPI_TalonSRX mLeft_Master;
    private WPI_TalonSRX mLeft_Slave0;
    private WPI_TalonSRX mLeft_Slave1;
    private WPI_TalonSRX mRight_Master;
    private WPI_TalonSRX mRight_Slave0;
    private WPI_TalonSRX mRight_Slave1;

    private DifferentialDrive mRoboDrive;

    public static DriveSys getInstance() {
        if (mDriveSys == null) {
            mDriveSys = new DriveSys();
        }
        return mDriveSys;
    }

    private DriveSys() {
        mLog = new LogWrapper(FileType.SUBSYSTEM, "DriveSystem", LoggerConstants.DRIVESYS_PERMISSION);
        mLeft_Master = new WPI_TalonSRX(DriveSysConstants.LEFT_TALON_MASTER);
        mLeft_Slave0 = new WPI_TalonSRX(DriveSysConstants.LEFT_TALON_SLAVE0);
        mLeft_Slave1 = new WPI_TalonSRX(DriveSysConstants.LEFT_TALON_SLAVE1);
        mRight_Master = new WPI_TalonSRX(DriveSysConstants.RIGHT_TALON_MASTER);
        mRight_Slave0 = new WPI_TalonSRX(DriveSysConstants.RIGHT_TALON_SLAVE0);
        mRight_Slave1 = new WPI_TalonSRX(DriveSysConstants.RIGHT_TALON_SLAVE1);

        mLeft_Slave0.follow(mLeft_Master);
        mLeft_Slave1.follow(mLeft_Master);
        mRight_Slave0.follow(mRight_Master);
        mRight_Slave1.follow(mRight_Master);

        mLeft_Master.setSensorPhase(DriveSysConstants.LEFT_TALON_MASTER_SENSOR_PHASE);
        mLeft_Slave0.setSensorPhase(DriveSysConstants.LEFT_TALON_SLAVE0_SENSOR_PHASE);
        mLeft_Slave1.setSensorPhase(DriveSysConstants.LEFT_TALON_SLAVE1_SENSOR_PHASE);
        mRight_Master.setSensorPhase(DriveSysConstants.RIGHT_TALON_MASTER_SENSOR_PHASE);
        mRight_Slave0.setSensorPhase(DriveSysConstants.RIGHT_TALON_SLAVE0_SENSOR_PHASE);
        mRight_Slave1.setSensorPhase(DriveSysConstants.RIGHT_TALON_SLAVE1_SENSOR_PHASE);

        // mLeft_Master.setInverted(DriveSysConstants.DRIVE_LEFT_INVERT);
        // mLeft_Slave0.setInverted(DriveSysConstants.DRIVE_LEFT_INVERT);
        mLeft_Slave1.setInverted(DriveSysConstants.DRIVE_LEFT_INVERT);
        // mRight_Master.setInverted(DriveSysConstants.DRIVE_RIGHT_INVERT);
        // mRight_Slave0.setInverted(DriveSysConstants.DRIVE_RIGHT_INVERT);
        // mRight_Slave1.setInverted(DriveSysConstants.DRIVE_RIGHT_INVERT);

        mLeft_Master.configOpenloopRamp(DriveSysConstants.DRIVE_CONFIG_OPEN_LOOP_RAMP,
                DriveSysConstants.DRIVE_TIME_OUT);
        mRight_Master.configOpenloopRamp(DriveSysConstants.DRIVE_CONFIG_OPEN_LOOP_RAMP,
                DriveSysConstants.DRIVE_TIME_OUT);

        mLeft_Master.setNeutralMode(DriveSysConstants.DRIVE_NEUTRAL_MODE);
        mRight_Master.setNeutralMode(DriveSysConstants.DRIVE_NEUTRAL_MODE);

        mRoboDrive = new DifferentialDrive(mLeft_Master, mRight_Master);

    }

    /***********************************************************
     * 
     *
     * Arcade Drive stuff
     * 
     * 
     ***********************************************************/

    public void arcadeDrive(double mag, double yaw) {
        // yaw is weird
        mRoboDrive.arcadeDrive(mag, -yaw, true);
        mLog.periodicPrint("Magnitude: " + mag + " yaw: " + yaw, 20);
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
        mLog.periodicDebug(30,"Yaw", yaw);
        if (yaw > RELATIVE_YAW_THRESHOLD) {
            arcadeDrive(0.0, yaw);
        } else {
            arcadeDrive(magnitude, yaw);
        }
    }

    public void initDefaultCommand() {

    }
}