

package team6072.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.constants.subsystems.IntakeSysConstants;

/**
 * Intake sys collects balls from the floor, moves them through the tunnel into the canister
 * It also has to moves balls down from the tower into the shooter
 * 
 * Elements:
 *  1. External arm that has
 *      Piston to lower and lift arm 
 *      Motor to drive intake wheels
 *  2. One motor in tunnel to drive balls back into canister, and reverse to drive into shooter
 *  3. One motor in canister to drive balls up or dowm
 * 
 * None of the motors need encoders
 */
public class IntakeSys implements Subsystem {


    private WPI_TalonSRX mIntakeMotor;
    private WPI_TalonSRX mBeltMotor;
    private WPI_TalonSRX mCanistorMotor;
    
}
