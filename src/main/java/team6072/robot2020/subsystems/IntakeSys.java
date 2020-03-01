
package team6072.robot2020.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.constants.subsystems.IntakeSysConstants;
import team6072.robot2020.constants.subsystems.PneumaticsConstants;

/**
 * Intake sys collects balls from the floor, moves them through the tunnel into the canister
 * It also has to moves balls down from the tower into the shooter
 * 
 * Elements:
 *  1. External arm that has
 *      Piston to lower and lift arm 
 *      Motor to drive intake wheels
 *  2. One motor in tunnel to drive balls back into canister, and reverse to drive into shooter
 *  3. One motor in canister to drive balls and reverse to drive balls into shooter
 *      Set canister motor as slave to belt motor
 * 
 * None of the motors need encoders
 * 
 * Commands:
 *      button to flip intake up and down
 *      trigger to run intake wheels
 *      button to run belt forward / reverse
 */
public class IntakeSys implements Subsystem {


    private LogWrapper mLog;

    private WPI_TalonSRX mIntakeMotor;
    private WPI_TalonSRX mBeltMotor;
    private WPI_TalonSRX mCanistorMotor;

    private DoubleSolenoid mIntakeSolenoid;

    private static IntakeSys mIntakeSys;
    

    public static IntakeSys getInstance() {
        if (mIntakeSys == null) {
            mIntakeSys = new IntakeSys();
        }
        return mIntakeSys;
    }


    private IntakeSys() {
        mLog = new LogWrapper(FileType.SUBSYSTEM, "Intake", LoggerConstants.INTAKE_PERMISSION);

        mIntakeMotor = new WPI_TalonSRX(IntakeSysConstants.INTAKE_TALON);
        mIntakeMotor.setInverted(IntakeSysConstants.INTAKE_TALON_INVERT);
        mIntakeMotor.setNeutralMode(IntakeSysConstants.INTAKE_TALON_NEUTRAL_MODE);

        mBeltMotor = new WPI_TalonSRX(IntakeSysConstants.BELT_TALON);
        mBeltMotor.setInverted(IntakeSysConstants.BELT_TALON_INVERT);
        mBeltMotor.setNeutralMode(IntakeSysConstants.BELT_TALON_NEUTRAL_MODE);

        mCanistorMotor = new WPI_TalonSRX(IntakeSysConstants.CAN_TALON);
        mCanistorMotor.setInverted(IntakeSysConstants.CAN_TALON_INVERT);
        mCanistorMotor.setNeutralMode(IntakeSysConstants.CAN_TALON_NEUTRAL_MODE);

        mCanistorMotor.follow(mBeltMotor);      // set canistor to follow belt

        mIntakeSolenoid = new DoubleSolenoid(PneumaticsConstants.PCMID, IntakeSysConstants.INTAKE_UP, 
                IntakeSysConstants.INTAKE_DOWN);
    }


    //  ------  flip intake up / down  -------------------

    public void setIntakeUp() {
        mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setIntakeDown() {
        mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }


    // ------  run intake wheels  ------------------------


    public void intakeWheelsIn() {
        mIntakeMotor.set(1);
    }


    public void intakeWheelsStop() {
        mIntakeMotor.set(0.0);
    }


    //  -----  run belt forward reverse

    public void beltMotorIn() {
        mBeltMotor.set(1);
    }

    public void beltMotorOut() {
        mBeltMotor.set(-1);
    }

    public void beltMotorStop() {
        mBeltMotor.set(0);
    }

}
