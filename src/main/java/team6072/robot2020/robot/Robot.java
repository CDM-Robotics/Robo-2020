/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robot2020.robot;

import java.io.*;
import java.util.ArrayList;
import java.util.logging.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Filesystem;
import team6072.robot2020.commands.drivesys.ArcadeDriveCmd;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.RobotTracker;
import team6072.robot2020.utility.logging.JLogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.utility.math.Angle2D;
import team6072.robot2020.utility.math.Position2D;
import team6072.robot2020.utility.math.Vector2D;
import team6072.robot2020.utility.thread.RunAndEndable;
import team6072.robot2020.subsystems.ColorSensorSys;
import team6072.robot2020.subsystems.DriveSys;
import team6072.robot2020.commands.drivesys.RelativeDriveCmd;
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
	private ArrayList<RunAndEndable> threads;
	private LogWrapper mLog;

	private JLogWrapper mJLog;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// initialize the Java logging system
		configLogging();
		mLog.alarm("------  Robot initialization  ------------");

		mScheduler = CommandScheduler.getInstance();

		ControlBoard.getInstance();
		DriveSys.getInstance();
		NavXSys.getInstance();
		NavXSys.getInstance().resetAll();
		ColorSensorSys.getInstance();
		RobotTracker.getInstance();

		// initializing all the independent threads
		threads = new ArrayList<RunAndEndable>();
		threads.add(RobotTracker.getInstance());
		// starts all threads
	}

	private void configLogging() {
		mLog = new LogWrapper(FileType.ROBOT, "Robot", team6072.robot2020.utility.logging.LogWrapper.Permission.ALL);
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
	}



	
	/**
	 * starts all the independent threads in the threads Array list
	 */
	private void startThreads() {
		for (int i = 0; i < threads.size(); i++) {
			Thread thread = new Thread(threads.get(i));
			thread.start();
		}
	}

	/**
	 * Autonomous code -----------------------------------------------------
	 */

	public void autonomousInit() {
		mLog.alarm("Autonomous");

		// Cancel all events
		mScheduler.cancelAll();

		// start independent Threads
		startThreads();

		// Schedule commands
	}

	@Override
	public void autonomousPeriodic() {
		Position2D position2d = RobotTracker.getInstance().getAbsolutePosition();
		mLog.periodicDebug(10, "X", position2d.getPositionVector2D().getX(), "Y",
				position2d.getPositionVector2D().getY(), "angle", position2d.getAngle2D().getDegrees());

		mScheduler.run();
	}

	/**
	 * Teleop code -------------------------------------------------------------
	 */

	public void teleopInit() {
		mLog.alarm("TelopInit");

		// Cancel all events
		mScheduler.cancelAll();

		// start independent Threads
		startThreads();

		// Schedule commands
		ArcadeDriveCmd arcadeDriveCmd = new ArcadeDriveCmd(ControlBoard.getInstance().mDriveStick);
		mScheduler.schedule(arcadeDriveCmd);
	}

	@Override
	public void teleopPeriodic() {
		try {
			mScheduler.run();
		} catch (Exception ex) {
			mJLog.severe(ex, "Robot.teleopPeriodic:  exception: " + ex.getMessage());
		}
	}

	/**
	 * Disabled init, occurs whenever the robot is in disable mode.
	 */
	public void disabledInit() {
		// disables all other independent threads being run
		endThreads();
	}
	
	/**
	 * ends all the independent threads in the threads Array list
	 */
	private void endThreads() {
		for (int i = 0; i < threads.size(); i++) {
			mLog.print("Ending Thread " + i);
			threads.get(i).end();
		}
	}
}
