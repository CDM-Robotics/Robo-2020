/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robot2020;

import edu.wpi.first.wpilibj.Joystick;
import team6072.robot2020.commands.ArcadeDriveCmd;
import team6072.robot2020.commands.RelativeDriveCmd;
import team6072.robot2020.constants.ControlBoardConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class ControlBoard {

    private static ControlBoard mControlBoard;
    public Joystick mJoystick0;
    public Joystick mJoystick1;

    public static ControlBoard getInstance() {
        if (mControlBoard == null) {
            mControlBoard = new ControlBoard();
        }
        return mControlBoard;
    }

    /**
     * Initializes all buttons and joystick controls to standard Commands
     * Note that this is to initialize commands that are teh same in Autonomous and Teleop
     * 
     */
    private ControlBoard() {
        
        mJoystick0 = new Joystick(ControlBoardConstants.JOYSTICK0);
        mJoystick1 = new Joystick(ControlBoardConstants.JOYSTICK1);
        // CommandScheduler.getInstance().schedule(new RelativeDriveCmd(mJoystick0));
    }

    private void MapCmdToBut(Joystick stick, int button, Command pressCmd, Command releaseCmd) {
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
    private void MapCmdToPovBut(Joystick stick, PovAngle angle, Command pressCmd, Command releaseCmd) {
        POVButton but = new POVButton(stick, angle.getAngle());
        if (pressCmd != null) {
            but.whenPressed(pressCmd);
        }
        if (releaseCmd != null) {
            but.whenReleased(releaseCmd);
        }
    }

}
