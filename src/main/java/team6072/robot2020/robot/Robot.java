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
			// get segment 0 delta vector, get curnposition vector added with the start
			// poosition vector on the segment
			// Then do the dot product of those vectors and divide that product with the
			// magnitude of the segment squared. Then take taht number and scale the segment
			// vector. then subtract that vector with the robotvector.

			// Test this bit of code line by line to make it work

			Vector2D seg = segments.get(0).getDelta();
			Vector2D robotPos = curnAbsolutePosition.getPositionVector2D();
			Vector2D start = segments.get(0).getStart();
			Vector2D robotVector = Vector2D.addVectors(start.inverse(), robotPos);

			double dotProduct = Vector2D.dotProduct(seg, robotVector);
			double percentOfSeg = dotProduct / (seg.getMagSquared());
			Vector2D scaledDownSeg = Vector2D.scaleVector(seg, percentOfSeg);
			Vector2D robotDisplacementVector = Vector2D.addVectors(scaledDownSeg.inverse(), robotVector);

			double robotDisplacementFromSegment = robotDisplacementVector.getMag();

			// do the same with hte next vector

			Vector2D seg2 = segments.get(1).getDelta();
			Vector2D startSeg2 = segments.get(1).getStart();
			Vector2D robotVectorS2 = Vector2D.addVectors(startSeg2.inverse(), robotPos);

			double dotProductS2 = Vector2D.dotProduct(seg2, robotVectorS2);
			double percentOfSegS2 = dotProductS2 / (seg2.getMagSquared());
			Vector2D scaledDownSegS2 = Vector2D.scaleVector(seg2, percentOfSegS2);
			Vector2D robotDisplacementVectorS2 = Vector2D.addVectors(scaledDownSegS2.inverse(), robotVectorS2);

			double robotDisplacementFromSegment2 = robotDisplacementVectorS2.getMag();

			if (robotDisplacementFromSegment > robotDisplacementFromSegment2) {
				segments.remove(0);
			}

			double lookAheadDistance = (DriveSys.getInstance().getLeftCurnVelInches()
					+ DriveSys.getInstance().getRightCurnVelInches()) / 2d;

			// test this loop
			int i = 0;
			while(i < segments.size() && lookAheadDistance > segments.get(i).getDistance()){
				lookAheadDistance = lookAheadDistance - segments.get(i).getDistance();
				i++;
			}

			// test this too
			double percentOfLookAheadVector = lookAheadDistance / segments.get(i).getDistance();
			Vector2D lookAheadVector = Vector2D.scaleVector(segments.get(i).getDelta(), percentOfLookAheadVector);
			Vector2D lookAheadPoint = Vector2D.addVectors(lookAheadVector, segments.get(i).getStart());
			
			Vector2D robotHeadVector = Vector2D.getVectorFromMagAndDegrees(1, curnAbsolutePosition.getAngle2D().getDegrees());
			Vector2D lookAheadPointVectorFromRobotPos = Vector2D.addVectors(lookAheadPoint, robotPos.inverse());

			// test which direction the robot should turn.
			
			// find the perpedicular vector of robotHead Vector
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
