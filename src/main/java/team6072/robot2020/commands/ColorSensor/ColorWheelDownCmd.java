package team6072.robot2020.commands.ColorSensor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team6072.robot2020.subsystems.ColorSensorSys;
import team6072.robot2020.subsystems.IntakeSys;

public class ColorWheelDownCmd extends CommandBase {

    private ColorSensorSys mColorSensorSys;

    /**
     * Creates a new IntakeWheelsInCmd.
     */
    public ColorWheelDownCmd() {
        mColorSensorSys = ColorSensorSys.getInstance();
        addRequirements(mColorSensorSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mColorSensorSys.setColorWheelDown();
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
