
package team6072.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.constants.subsystems.ClimberSysConstants;
import team6072.robot2020.constants.subsystems.PneumaticsConstants;

/**
 * Climber sys deploys a hook to attach to the seesaw bar
 * 
 * Elements:
 *  1. Piston to lift the hook to position
 *  2. Motor to spool in the rope to climb, once hook is attached to bar
 */
public class ClimberSys implements Subsystem {

    private LogWrapper mLog;

    private WPI_TalonSRX mClimberMotor;

    private DoubleSolenoid mClimberSolenoid;

    private static ClimberSys mClimberSys;

    public static ClimberSys getInstance() {
        if (mClimberSys == null) {
            mClimberSys = new ClimberSys();
        }
        return mClimberSys;
    }

    private ClimberSys() {
        mLog = new LogWrapper(FileType.SUBSYSTEM, "Intake", LoggerConstants.INTAKE_PERMISSION);

        mClimberMotor = new WPI_TalonSRX(ClimberSysConstants.INTAKE_TALON);
        mClimberMotor.setInverted(ClimberSysConstants.INTAKE_TALON_INVERT);
        mClimberMotor.setNeutralMode(ClimberSysConstants.INTAKE_TALON_NEUTRAL_MODE);

    }

}
