/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robot2020;

import java.io.*;
import java.util.logging.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import team6072.robot2020.commands.ArcadeDriveCmd;
import team6072.robot2020.logging.LogWrapper;
import team6072.robot2020.logging.JLogWrapper;
import team6072.robot2020.logging.LogWrapper.FileType;
import team6072.robot2020.subsystems.DriveSys;
import team6072.robot2020.commands.RelativeDriveCmd;
import team6072.robot2020.subsystems.NavXSys;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private CommandScheduler mScheduler;
	private LogWrapper mLog;

	private JLogWrapper mJLog;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		mLog = new LogWrapper(FileType.ROBOT, "Robot", team6072.robot2020.logging.LogWrapper.Permission.ALL);
		// initialize the Java logging system
		try {
			File dir = Filesystem.getDeployDirectory();
			String logFile = "NOTFOUND";
			if (dir.isDirectory()) {
				logFile = dir.getAbsolutePath() + "/logging.properties";
			}
			System.out.println("**********  logConfig: " + logFile + "  *********************");
			FileInputStream configFile = new FileInputStream(logFile);
			LogManager.getLogManager().readConfiguration(configFile);
		} catch (IOException ex) {
			System.out.println("WARNING: Could not open logging configuration file");
			System.out.println("WARNING: Logging not configured (console output only)");
		}
		mJLog = new JLogWrapper(Robot.class.getName());
		mLog.alarm("------  Robot initialization  ------------");
		mScheduler = CommandScheduler.getInstance();

		ControlBoard.getInstance();
		DriveSys.getInstance();
		NavXSys.getInstance();
	}

	public void disabledInit() {
	}

	/**
	 * Autonomous code -----------------------------------------------------
	 */

	public void autonomousInit() {
		mScheduler.cancelAll();
		RelativeDriveCmd relativeDriveCmd = new RelativeDriveCmd(ControlBoard.getInstance().mJoystick0);
		mScheduler.schedule(relativeDriveCmd);
		mLog.alarm("Autonomous");
		NavXSys.getInstance().resetAll();
	}

	@Override
	public void autonomousPeriodic() {
		mScheduler.run();
	}

	/**
	 * Teleop code -------------------------------------------------------------
	 */

	public void teleopInit() {
		mScheduler.cancelAll();
		ArcadeDriveCmd arcadeDriveCmd = new ArcadeDriveCmd(ControlBoard.getInstance().mJoystick0);
		mScheduler.schedule(arcadeDriveCmd);
		mLog.alarm("TelopInit");
		NavXSys.getInstance().resetAll();
	}

	@Override
	public void teleopPeriodic() {
		try {
			mScheduler.run();
		} catch (Exception ex) {
			mJLog.severe(ex, "Robot.teleopPeriodic:  exception: " + ex.getMessage());
		}
	}

}
