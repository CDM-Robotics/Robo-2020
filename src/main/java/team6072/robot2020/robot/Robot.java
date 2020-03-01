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
import team6072.robot2020.utility.movement.RobotTracker;
import team6072.robot2020.utility.movement.WatchDogMaster;
import team6072.robot2020.utility.logging.JLogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.utility.math.Angle2D;
import team6072.robot2020.utility.math.Position2D;
import team6072.robot2020.utility.math.Vector2D;
import team6072.robot2020.utility.math.Segment;
import team6072.robot2020.utility.RunAndEndable;
import team6072.robot2020.subsystems.ColorSensorSys;
import team6072.robot2020.subsystems.DriveSys;
import team6072.robot2020.commands.drivesys.RelativeDriveCmd;
import team6072.robot2020.subsystems.NavXSys;
import team6072.robot2020.commands.PurePursuitCmd;

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

		NavXSys.getInstance();
		ControlBoard.getInstance();
		DriveSys.getInstance();
		ColorSensorSys.getInstance();

		// initializing all the independent threads
		threads = new ArrayList<RunAndEndable>();
		threads.add(RobotTracker.getInstance());
		threads.add(WatchDogMaster.getInstance());

		// Set starting points
		NavXSys.getInstance().resetAll(); // reset navx
		RobotTracker.getInstance().setCurrentPosition(new Vector2D()); // set starting point on XY plane
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

		mLog.alarm("Autonomous");
		Vector2D p1 = new Vector2D(4 * 12, 2 * 12);
		Vector2D p2 = new Vector2D(7 * 12, 1 * 12);

		Segment s1 = new Segment(RobotTracker.getInstance().getAbsolutePosition().getPositionVector2D(), p1);
		Segment s2 = new Segment(p1, p2);

		ArrayList<Segment> segments = new ArrayList<Segment>();

		boolean finished = false;

		while (!finished) {
			Position2D curnAbsolutePosition = RobotTracker.getInstance().getAbsolutePosition();
			Vector2D robotPos = curnAbsolutePosition.getPositionVector2D();

			// Path function - updatePath
			if (segments.size() > 1) {
				double robotDisplacementFromSegment1 = segments.get(0).getPerpendicularDistance(robotPos);
				double robotDisplacementFromSegment2 = segments.get(1).getPerpendicularDistance(robotPos);
				if (robotDisplacementFromSegment1 > robotDisplacementFromSegment2) {
					segments.remove(0);
				}
			}

			// Path Function - getLookAheadDistance
			double lookAheadDistance = (DriveSys.getInstance().getLeftCurnVelInches()
					+ DriveSys.getInstance().getRightCurnVelInches()) / 2d;
			lookAheadDistance = lookAheadDistance + segments.get(0).getParrellelDistance(robotPos);
			int i = 0;
			while (i < segments.size() && lookAheadDistance > segments.get(i).getDistance()) {
				lookAheadDistance = lookAheadDistance - segments.get(i).getDistance();
				i++;
			}

			// Path function - getLookAheadPoint
			double percentOfLookAheadVector = lookAheadDistance / segments.get(i).getDistance();
			Vector2D lookAheadVector = segments.get(i).getDelta().scaleVector(percentOfLookAheadVector);
			Vector2D lookAheadPoint = lookAheadVector.translateBy(segments.get(i).getStart());

			// Path Function - getRadius
			Vector2D robotHeadVector = Vector2D.fromMagAndAngle(1,
					new Angle2D(curnAbsolutePosition.getAngle2D().getDegrees()));
			Vector2D lookAheadPointVectorFromRobotPos = lookAheadPoint.translateBy(robotPos.inverse());

			Angle2D robotHead = robotHeadVector.getAngle();
			Angle2D lookAheadHeading = lookAheadPointVectorFromRobotPos.getAngle();

			double robotHeadingDegrees = robotHead.getDegrees();
			double lookAheadHeadingDegrees = lookAheadHeading.getDegrees();
			boolean direction; // true is right and left
			if(Math.abs(robotHeadingDegrees) > 180){
				if(robotHeadingDegrees > 0){
					robotHeadingDegrees -= 360;
				} else {
					robotHeadingDegrees += 360;
				}
			}
			if(Math.abs(lookAheadHeadingDegrees) < 180){
				if (lookAheadHeadingDegrees > 0) {
					lookAheadHeadingDegrees -= 360;
				} else {
					lookAheadHeadingDegrees += 360;
				}
			}
			
			if((lookAheadHeadingDegrees - robotHeadingDegrees) > 0){
				robotHeadVector.rotateBy(new Angle2D(90));
			} else {
				robotHeadVector.rotateBy(new Angle2D(-90));
			}
			
			// reduce lookAheadPointVectorFromRobotPos by 2

			// find the perpendicular line of lookAheadPointVectorFromRobotPos

			// find inntercection of those two lines

			// get vector from robot pos to that point, to get radius
			// get two radii
			// caluculate speed at those points
			// set motors

		}

	}

	@Override
	public void autonomousPeriodic() {
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
			// mLog.periodicDebug(10, "Left", DriveSys.getInstance().getLeftCurnPosInches(),
			// "Right", DriveSys.getInstance().getRightCurnPosInches());
			Position2D position = RobotTracker.getInstance().getAbsolutePosition();
			mLog.periodicDebug(10, "X", position.getPositionVector2D().getX(), "Y",
					position.getPositionVector2D().getY(), "angle", position.getAngle2D().getDegrees());
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
