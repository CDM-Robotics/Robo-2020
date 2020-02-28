package team6072.robot2020.commands.ColorSensor;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.Command;

import team6072.robot2020.subsystems.ColorSensorSys;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.constants.logging.LoggerConstants;



public class RotateCmd extends CommandBase {


  private ColorSensorSys  mColorSys;
  private LogWrapper mLog;


  /**
   * Ask the ColorSensorSys to rotate the wheel between 3 and 5 times
   * Need to alert drive station if the wheel is not turning (i.e. drive wheels have not made contact with color wheel)
   * @param sys
   */
  public RotateCmd(ColorSensorSys sys) {
    mColorSys = sys;
    addRequirements(mColorSys);
    mLog = new LogWrapper(FileType.COMMAND, "RotateCS", LoggerConstants.ROTATECS_CMD);
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    mColorSys.startRotateCmd();
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    mColorSys.execRotateCmd();
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return mColorSys.isRotateFinished();
  }


  // Called once after isFinished returns true
  // @Override
  // public void end() {
  // }


  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  // @Override
  // public void interrupted() {
  // }


}
