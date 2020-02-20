package team6072.robot2020.commands.drivesys;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import team6072.robot2020.subsystems.DriveSys;
import team6072.robot2020.utility.LogitechJoystick;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.constants.commands.RelativeDriveCmdConstants;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.movement.pid.datasources.NavXSource;
import team6072.robot2020.utility.movement.pid.datasources.NavXSource.NavXDataTypes;
import java.util.HashSet;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Relative drive is a drive mode that changes the role of the driving joystick.  
 * The joystick no longer drives relative to the robot, it drives relative to the directions of the field.
 * This means the robot will now turn automatically, making the driver's role to simply guiide net movements.
 * Let me explain.
 * If you push the joystick forward using Arcade drive, you would expect that the robot would drive forward 
 * no matter which direction forward is.  For example, if the robot is turned 45 degrees but you push
 * forward with the joystick, the robot will still drive at a 45 degree angle.
 * HOWEVER, with relative drive if the robot was turned 45 degrees but then you pushed the joystick forward,
 * the robot would first turn to face the 0 degrees direction and start driving forward ONLY when the robot was 
 * facing the right direction. 
 * another example.  
 * Say the robot was backwards on the field (facing the drivers) and the driver wanted the robot to turn around
 * (face the opponent drivers) and start driving forward.  With relative drive, all the driver has to do is push the joystick
 * in the forward direction.  The robot would then see that its current orientation is at 180 degrees and would turn around
 * to be facing 0 degrees.  Then when it has finished turning and it is facing the 0 degrees direction, ]
 * the robot would then start driving forward.
 * nuf sed.
 */
public class RelativeDriveCmd implements Command {

    private LogitechJoystick mStick;
    private DriveSys mDriveSys;
    private double mLastValidJoystickTarget = 0;
    private LogWrapper mLog;
    private NavXSource mAccumulatedYawSource;

    /**
     * This is the constructor for Relative drive, takes a josytick as a parameter
     * to pair it to the relatvie drive command
     * 
     * @param stick This varaible is the joystick that will be paired to the
     *              Relative Drive command
     */
    public RelativeDriveCmd(LogitechJoystick stick) {
        mLog = new LogWrapper(FileType.COMMAND, "RelativeDriveCmd", LoggerConstants.RELATIVE_DRIVE_CMD);
        // requires(DriveSys.getInstance());
        mDriveSys = DriveSys.getInstance();
        mStick = stick;
    }

    public Set<Subsystem> getRequirements(){
        HashSet<Subsystem> requirements = new HashSet<Subsystem>();
        requirements.add(mDriveSys);
        return requirements;
    }

    /**
     * This initializes the navX sources so that the data can be collected are relayed to the PID,
     * The function creates a new NavXYaw source as its source of data and then executes the initRelativeDrive 
     * function from the drivesystem.
     */
    public void initialize() {
        mAccumulatedYawSource = new NavXSource(NavXDataTypes.TOTAL_YAW);
        mDriveSys.initRelativeDrive();
    }

    /**
     * This is the fun part
     * 
     * The first part of the code recieves y and x information from the joystick, adjusting the values as necessary
     * Then calculates magnitude or the speed of the robot
     * The code then calculates the target angle and validates it through certian constant thresholds
     * Then it converts the target angle into the closest congruent angle to the robot's current angle position
     * Then it sends that angle to the drivesystem to engage the PID
     * 
     */
    public void execute() {
        // compute angle from joysticks
        double y = mStick.getInvertedY();
        double x = mStick.getX();
        y = -y; // adjusting magnitude as the joystick's forward position is negative.  Which is stupid
        mLog.periodicDebug(10, "Y", y, "X", x);
        // calculate the magnitude of speed to drive with the pathagorean theorum
        //      This magnitude will be the speed of the robot
        double magnitude = Math.sqrt((y * y) + (x * x));
        double joystickTarget = 0.0;

        // This is the part where the code calculates the target angle at which to turn the robot
        try {
            y = y / magnitude;
        } catch (Exception err) {
            mLog.warning("The magnitude of the joystick is zero.  If you want that then it is fine, but just a warning.");
        }
        // Sees if the magnitude is large enough to warrant turning the robot
        // checks to see if the joystick is being intentionally pushed
        if (magnitude > RelativeDriveCmdConstants.SET_ANGLE_THRESHOLD) {
            // This code figures out if the joystick is being pushed to the right or the left
            if (x < 0) {
                // Calculation time!!!
                joystickTarget = -Math.acos(y); // calculates the desired yaw of the robot in radians
                joystickTarget = ((joystickTarget * 360) / (2 * Math.PI)) + 360.0; // converts to degrees
                mLastValidJoystickTarget = joystickTarget; // makes it the new valid target
            } else {
                joystickTarget = Math.acos(y);
                joystickTarget = ((joystickTarget * 360) / (2 * Math.PI));
                mLastValidJoystickTarget = joystickTarget;
            }
        }

        // This is the tricky part
        // the angle we have right now is somewhere between 0 and 360
        // BUt the angle the robot is at could be anywhere from -infinity to +infinity
        // SOOO the following code translates our target angle into an angle that the robot can reasonably achieve
        //       For example, if our target is at 270 degrees and the robot is at 770 degrees
        //          it does not need to move 500 degrees to be in the right place.
        //          This code compensates by making our 270 degrees into 630 degrees so the robot does  
        //          not have to turn like crazy to be in the riht place.

        // This part calculates if it will turn to the left or the right
        int numOfRobotRevolutions = (int) ((mAccumulatedYawSource.getData() / 360.0)); // First we calculate how many revolutions the robot has already made
        double currentRevPercent = (mAccumulatedYawSource.getData() / 360.0) % 1.0; // from -.9999 to +.99999
        double currentRevPosition = currentRevPercent * 360.0; // Then we calculate where the robot is in its revolution from -359.99 to 359.99
        double err = Math.abs(mLastValidJoystickTarget - currentRevPosition); // This calculates the error between the current position and the target. This could be anywhere from > 180 to near 720 degrees

        // In this method I have made the situation down to any one of 2 possible scenarios
        // number 1 is that the target angle and the current position are very close to each other and dont have to be moved at all
        //      in this case the if statement below is ignored and it moves on
        // number 2 is that the target angle and the current position are very far apart,
        //      this occurs if the target angle and the current position are on different revolutions, such as 270 and 45 
        //      in this example the fastest route to the target is by traveling 225 degrees which is not efficient 
        //      Therefore the if statement runs and attempts, through trial and error, to see if that error can be fixed if 
        //      the target is moved up or down 360 degrees.
        if (err > 180.0) {
            if (Math.abs((mLastValidJoystickTarget + 360) - currentRevPosition) < 180) {
                mLastValidJoystickTarget = mLastValidJoystickTarget + 360;
            } else if (Math.abs((mLastValidJoystickTarget - 360) - currentRevPosition) < 180) {
                mLastValidJoystickTarget = mLastValidJoystickTarget - 360;
            }
        }
        // i just realized i did this in an extremely stupid way so I am going to come back to this later when I can test new code
        mLastValidJoystickTarget = mLastValidJoystickTarget + numOfRobotRevolutions * 360.0;
        // mLog.periodicDebug(30, "Y", y);
        // mLog.periodicDebug(10, "Y", mStick.getY(), "X", mStick.getX(), "Magnitude", magnitude, "TargetAngle", mLastValidJoystickTarget);
        // mLog.periodicPrint("magnitude: " + magnitude + ", mPerviousAngle: " + mLastValidJoystickTarget + ", mYaw: "
        //         + mYawSource.getData() + ", mAccumulatedYaw: " + mAccumulatedYawSource.getData(), 25);

        mDriveSys.executeRelativeDrive(mLastValidJoystickTarget, -magnitude);
    }

    public boolean isFinished() {
        return false;
    }

}