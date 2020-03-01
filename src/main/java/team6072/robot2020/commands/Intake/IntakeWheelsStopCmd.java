
package team6072.robot2020.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team6072.robot2020.subsystems.IntakeSys;


public class IntakeWheelsStopCmd extends CommandBase {

  private IntakeSys mIntakeSys;

  /**
   * Creates a new IntakeWheelsInCmd.
   */
  public IntakeWheelsStopCmd() {
    mIntakeSys = IntakeSys.getInstance();
    addRequirements(mIntakeSys);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntakeSys.intakeWheelsStop();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
