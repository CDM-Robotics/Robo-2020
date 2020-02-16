package team6072.robot2020.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.pid.MyPIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import team6072.robot2020.datasources.NavXSource;
import team6072.robot2020.datasources.NavXSource.NavXDataTypes;
import team6072.robot2020.constants.subsystems.DriveSysConstants;
import team6072.robot2020.logging.LogWrapper;
import team6072.robot2020.logging.LogWrapper.FileType;
import team6072.robot2020.commands.drivesys.*;
import team6072.robot2020.ControlBoard;



public class DriveSys implements Subsystem {

    private LogWrapper mLog;

    private WPI_TalonFX mLeft_Master;
    private WPI_TalonFX mLeft_Slave0;

    private WPI_TalonFX mRight_Master;
    private WPI_TalonFX mRight_Slave0;

    private DifferentialDrive mRoboDrive;

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

        mLeft_Slave0.follow(mLeft_Master);
        mRight_Slave0.follow(mRight_Master);
        
        mLeft_Master.setInverted(DriveSysConstants.DRIVE_LEFT_INVERT);
        mLeft_Slave0.setInverted(InvertType.FollowMaster);
        mRight_Master.setInverted(DriveSysConstants.DRIVE_RIGHT_INVERT);
        mRight_Slave0.setInverted(InvertType.FollowMaster);

        mLeft_Master.setSensorPhase(DriveSysConstants.LEFT_TALON_MASTER_SENSOR_PHASE);
        //mLeft_Slave0.setSensorPhase(DriveSysConstants.LEFT_TALON_SLAVE0_SENSOR_PHASE);
        mRight_Master.setSensorPhase(DriveSysConstants.RIGHT_TALON_MASTER_SENSOR_PHASE);
        //mRight_Slave0.setSensorPhase(DriveSysConstants.RIGHT_TALON_SLAVE0_SENSOR_PHASE);

        mLeft_Master.configOpenloopRamp(DriveSysConstants.DRIVE_CONFIG_OPEN_LOOP_RAMP,
                DriveSysConstants.DRIVE_TIME_OUT);
        mRight_Master.configOpenloopRamp(DriveSysConstants.DRIVE_CONFIG_OPEN_LOOP_RAMP,
                DriveSysConstants.DRIVE_TIME_OUT);

        mLeft_Master.setNeutralMode(DriveSysConstants.DRIVE_NEUTRAL_MODE);
        mRight_Master.setNeutralMode(DriveSysConstants.DRIVE_NEUTRAL_MODE);

        mRoboDrive = new DifferentialDrive(mLeft_Master, mRight_Master);
    }


    public void initDefaultCommand() {
        setDefaultCommand(new ArcadeDriveCmd(ControlBoard.getInstance().mDriveStick));
    }

    /***********************************************************
     * 
     *
     * Arcade Drive stuff
     * 
     * 
     ***********************************************************/

    // private double priorPosition = 0;
    // private double curnposition = 0;
    // private boolean first = false;
    public void arcadeDrive(double mag, double yaw) {
        // yaw is weird
        mRoboDrive.arcadeDrive(mag, -yaw, true);
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
        mLog.periodicDebug(30,"Yaw", yaw);
        if (yaw > RELATIVE_YAW_THRESHOLD) {
            arcadeDrive(0.0, yaw);
        } else {
            arcadeDrive(magnitude, yaw);
        }
    }


}