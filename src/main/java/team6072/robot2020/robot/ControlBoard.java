/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robot2020.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;


import team6072.robot2020.utility.LogitechJoystick;
import team6072.robot2020.constants.ControlBoardConstants;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.utility.logging.LogWrapper.Permission;

import team6072.robot2020.subsystems.ColorSensorSys;
import team6072.robot2020.commands.ColorSensor.*;
import team6072.robot2020.commands.Intake.*;




/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class ControlBoard {

    // logitech gamepad buttons
    public static int LOGITECH_BUT_A = 1;
    public static int LOGITECH_BUT_B = 2;
    public static int LOGITECH_BUT_X = 3;
    public static int LOGITECH_BUT_Y = 4;
    public static int LOGITECH_BUT_LEFT = 5;
    public static int LOGITECH_BUT_RIGHT = 6;

    // extreme buttons
    // Y-axis - forward and back
    // X-axis - left and right
    // Z-axis - twist
    // hub is POV?
    public static int EXTREME_BUT_TRIGGER = 1;
    public static int EXTREME_BUT_THUMB = 2;
    public static int EXTREME_BUT_LEFT_TOP = 5;
    public static int EXTREME_BUT_LEFT_BOT = 3;
    public static int EXTREME_BUT_RIGHT_TOP = 6;
    public static int EXTREME_BUT_RIGHT_BOT = 4;
    public static int EXTREME_BUT_7 = 7;
    public static int EXTREME_BUT_8 = 8;
    public static int EXTREME_BUT_9 = 9;
    public static int EXTREME_BUT_10 = 10; // not working?
    public static int EXTREME_BUT_11 = 11; // not working?
    public static int EXTREME_BUT_12 = 12;

    private static ControlBoard mControlBoard;

    // drive stick is used for driving robot
    private static int DRIVE_USB_PORT = 0;
    public LogitechJoystick mDriveStick;

    // control stick is used for elevator, intake
    private static int CONTROL_USB_PORT = 1;
    public LogitechJoystick mControlStick;

    public static ControlBoard getInstance() {
        if (mControlBoard == null) {
            mControlBoard = new ControlBoard();
        }
        return mControlBoard;
    }


    /**
     * Initializes all buttons and joystick controls to standard Commands
     * Note that this is to initialize commands that are the same in Autonomous and Teleop
     * 
     */
    private ControlBoard() {
        mDriveStick = new LogitechJoystick(DRIVE_USB_PORT);
        mControlStick = new LogitechJoystick(CONTROL_USB_PORT);
        // CommandScheduler.getInstance().schedule(new RelativeDriveCmd(mJoystick0));


        // Drive Stick Commands --------------------------------------



        // Control Stick Commands ------------------------------------

        MapCmdToBut(mControlStick, EXTREME_BUT_TRIGGER, new IntakeWheelsInCmd(),new IntakeWheelsStopCmd());
        MapCmdToBut(mControlStick, EXTREME_BUT_7, new BeltWheelsInCmd(), null);
        MapCmdToBut(mControlStick, EXTREME_BUT_8, new BeltWheelsOutCmd(), null);
        MapCmdToBut(mControlStick, EXTREME_BUT_12, new RotateCmd(ColorSensorSys.getInstance()), null);


    }




    /**
     * Map a button to one or two commands
     * @param stick
     * @param button
     * @param pressCmd      command to exec when pressed
     * @param releaseCmd    command to exec when released (may be null)
     */
    private void MapCmdToBut(LogitechJoystick stick, int button, Command pressCmd, Command releaseCmd) {
        JoystickButton but = new JoystickButton(stick, button);
        if (pressCmd != null) {
            but.whenPressed(pressCmd);
        }
        if (releaseCmd != null) {
            but.whenReleased(releaseCmd);
        }
    }


    public enum PovAngle {
        Deg_000(0), Deg_045(45), Deg_090(90), Deg_135(135), Deg_180(180), Deg_225(225), Deg_270(270), Deg_315(315);

        private int mAngle;

        PovAngle(int angle) {
            mAngle = angle;
        }

        public int getAngle() {
            return mAngle;
        }
    }


    /**
     * See PovButton here:
     * http://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/buttons/POVButton.html
     * 
     * PovButton is the small rotating button on top of the joystick
     */
    private void MapCmdToPovBut(LogitechJoystick stick, PovAngle angle, Command pressCmd, Command releaseCmd) {
        POVButton but = new POVButton(stick, angle.getAngle());
        if (pressCmd != null) {
            but.whenPressed(pressCmd);
        }
        if (releaseCmd != null) {
            but.whenReleased(releaseCmd);
        }
    }

}
