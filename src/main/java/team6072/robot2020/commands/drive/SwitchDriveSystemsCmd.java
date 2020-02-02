package team6072.robot2020.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Set;
import edu.wpi.first.wpilibj2.command.Subsystem;
import team6072.robot2020.logging.LogWrapper;
import team6072.robot2020.logging.LogWrapper.FileType;
import team6072.robot2020.logging.LogWrapper.Permission;

public class SwitchDriveSystemsCmd implements Command {

    private LogWrapper mLog;
    private Joystick mJoystick;
    private boolean mWhichDriveCmd; // This is a boolean that represents which drive command is currently running -
                                    // true is RelativeDrive - false is ArcadeDrive

    /**
     * the boolean represents which drive command is currently running 
     * true is RelativeDrive - false is ArcadeDrive
     */
    public SwitchDriveSystemsCmd(boolean whichDrive, Joystick joystick) {
        mLog = new LogWrapper(FileType.COMMAND, "Switch DriveSystems Cmd", Permission.ALL);
        mLog.print("SwitchDrivesystemsCmd constructor");
        mJoystick = joystick;
        mWhichDriveCmd = whichDrive;
    }

    public Set<Subsystem> getRequirements() {
        return null;
    }

    public void initialize() {
        mLog.alarm("Initializing Switch Drive SYstems comd");
        if(mWhichDriveCmd){
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().schedule(new ArcadeDriveCmd(mJoystick));
            mWhichDriveCmd = false;
        } else {
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().schedule(new RelativeDriveCmd(mJoystick));
            mWhichDriveCmd = true;
        }

    }

    public boolean isFinished() {
        return true;
    }

}