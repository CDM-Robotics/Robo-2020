/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robot2020;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team6072.robot2020.logging.LogWrapper;
import team6072.robot2020.logging.LogWrapper.FileType;
import team6072.robot2020.subsystems.ColorSensorSys;
import team6072.robot2020.subsystems.DriveSys;
import team6072.robot2020.subsystems.FMSSys;
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

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    mLog = new LogWrapper(FileType.ROBOT, "Robot", team6072.robot2020.logging.LogWrapper.Permission.ALL);
    mScheduler = CommandScheduler.getInstance();

    mScheduler.cancelAll();

    DriveSys.getInstance();
    NavXSys.getInstance();
    ColorSensorSys.getInstance();
  }

  public void disabledInit() {
  }

  public void teleopInit() {
    ControlBoard.getInstance();
    NavXSys.getInstance().resetAll();
  }

  @Override
  public void teleopPeriodic() {
    // mScheduler.run();
    DriveSys.getInstance().arcadeDrive(0, 0);
    // mLog.periodicDebug(25, "Blue", ColorSensorSys.getInstance().getBlue(), "Red", ColorSensorSys.getInstance().getRed(),
    //     "Green", ColorSensorSys.getInstance().getGreen(), "Distance", ColorSensorSys.getInstance().getDistance());
    mLog.periodicPrint("Color: " + ColorSensorSys.getInstance().matchColor().toString(), 25);

    // mLog.periodicDebug(25, "Red", ColorSensorSys.getInstance().getColor().red, "Green", ColorSensorSys.getInstance().getColor().green, "Blue", ColorSensorSys.getInstance().getColor().blue);
  }

}
