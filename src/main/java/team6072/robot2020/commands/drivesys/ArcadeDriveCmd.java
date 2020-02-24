package team6072.robot2020.commands.drivesys;

import java.util.HashSet;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import team6072.robot2020.utility.LogitechJoystick;

import team6072.robot2020.subsystems.DriveSys;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.constants.subsystems.DriveSysConstants;

/**
 * This is the main driving method used by FRC it consists of moving and turning
 * relative to the robot at all times This means pushing right or left on the
 * joystick will result in turning right or left respectively while pushing
 * forward will make the robot drive in forward
 */
public class ArcadeDriveCmd implements Command {

    private LogitechJoystick mStick;
    private LogWrapper mLog;
    private DriveSys mDriveSys;

    /**
     * Specify the the command requires the DriveSys subsystem
     */
    public ArcadeDriveCmd(LogitechJoystick stick) {
        // requires(DriveSys.getInstance());
        mStick = stick;
        mLog = new LogWrapper(FileType.COMMAND, "ArcadeDrive", LoggerConstants.ARCADE_DRIVE_CMD);
        mDriveSys = DriveSys.getInstance();
    }

    public Set<Subsystem> getRequirements() {
        HashSet<Subsystem> requirements = new HashSet<Subsystem>();
        requirements.add(mDriveSys);
        return requirements;
    }

    /**
     * Execute is called by the scheduler until the command returns finished or the
     * OI stops requesting - for example if the whileHeld() button command is used
     */
    public void execute() {
        double mag = mStick.getInvertedY();
        double yaw = mStick.getX();

        // mag = -mag;// y comes out from stick as negative when going forward, so
        // convert
        yaw = yaw * 0.8; // reduce sensitivity on turn
        mag = mag * DriveSysConstants.MAX_DRIVE_SPEED;
        yaw = yaw * DriveSysConstants.MAX_DRIVE_SPEED;

        mLog.periodicDebug(20, "Y", mag, "X", yaw);
        mDriveSys.arcadeDrive(mag, yaw);
    }

    /**
     * @return Return true when command is completed
     */
    @Override
    public boolean isFinished() {
        return false;
    }

}