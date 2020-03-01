package team6072.robot2020.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C.Port;

import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.FMSUtility;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.constants.subsystems.ColorSysConstants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import team6072.robot2020.constants.subsystems.PneumaticsConstants;


/**
 * Color sensor is responsilbe for 
 *  1. detecting the color of the wheel 
 *  2. driving the wheel to the correct position
 * 
 * Elements are: 
 *  1. color sensor 
 *  2. piston to move the sensor and driving wheel into position 
 *  3. motor with encoder to drive the wheel
 * 
 * Each CONTROL PANEL has two (2) requirements in order to ENERGIZE the SHIELD
 * GENERATOR, see CONTROL PANEL Scoring for complete details on game play. 
 * 
 * ROTATION CONTROL: Rotate CONTROL PANEL at least three (3) (but no more than
 * five (5)) complete revolutions in the same direction. If the CONTROL PANEL is
 * rotated more than five (5) complete revolutions, the count resets to zero
 * (0). The TRENCH light turns on once Stage 2 CAPACITY is reached (i.e. the
 * CONTROL PANEL is ready for ROTATION CONTROL).
 * 
 * POSITION CONTROL: Rotate CONTROL PANEL so a specified color aligns with the
 * sensor for at least five (5) seconds. Once either ALLIANCE reaches Stage 3
 * CAPACITY, FMS relays a specified color (randomly selected by FMS and one (1)
 * of the three (3) colors not currently read by the ALLIANCE’S TRENCH color
 * sensor) to all OPERATOR CONSOLES simultaneously. The specified color may not
 * be the same for both ALLIANCES. See Table 3-4 for details on how the TRENCH
 * light is used during POSTION CONTROL.
 */
public class ColorSensorSys implements Subsystem {


    private static ColorSensorSys mColorSensorSys;

    private Color mBlue;
    private Color mRed;
    private Color mYellow;
    private Color mGreen;

    private LogWrapper mLog;
    private ColorSensorV3 mSensor;
    private ColorMatch mColorMatch;

    private WPI_TalonSRX mCSTalon;
    
    private DoubleSolenoid mColorWheelSolenoid;
    
    public static ColorSensorSys getInstance() {
        if (mColorSensorSys == null) {
            mColorSensorSys = new ColorSensorSys();
        }
        return mColorSensorSys;
    }



    private ColorSensorSys() {
        mLog = new LogWrapper(FileType.SUBSYSTEM, "Color Sensor", LoggerConstants.COLOR_SENSOR_PERMISSION);
        mSensor = new ColorSensorV3(Port.kOnboard);

        mColorMatch = new ColorMatch();

        mBlue = ColorMatch.makeColor(0.180, 0.443, 0.377);
        mGreen = ColorMatch.makeColor(0.216, 0.516, 0.269);
        mYellow = ColorMatch.makeColor(0.300, 0.527, .174);
        mRed = ColorMatch.makeColor(0.360, 0.420, 0.220);

        mColorMatch.addColorMatch(mBlue);
        mColorMatch.addColorMatch(mGreen);
        mColorMatch.addColorMatch(mYellow);
        mColorMatch.addColorMatch(mRed);

        mCSTalon = new WPI_TalonSRX(ColorSysConstants.CS_TALON);
        mCSTalon.setInverted(ColorSysConstants.CS_TALON_INVERT);
        mCSTalon.setNeutralMode(ColorSysConstants.CS_TALON_NEUTRAL_MODE);

        mColorWheelSolenoid = new DoubleSolenoid(PneumaticsConstants.PCMID, ColorSysConstants.COLOR_UP,
                ColorSysConstants.COLOR_DOWN);
    }



    private int getDistance() {
        return mSensor.getProximity();
    }



    /**
     * Read the current sensor color and translate to the FMS color
     * @return
     */
    private FMSUtility.Color getSensorFMSColor() {
        Color color = mSensor.getColor();
        ColorMatchResult result = mColorMatch.matchClosestColor(color);
        if (result.color == mBlue) {
            return FMSUtility.Color.Blue;
        } else if (result.color == mGreen) {
            return FMSUtility.Color.Green;
        } else if (result.color == mYellow) {
            return FMSUtility.Color.Yellow;
        } else if (result.color == mRed) {
            return FMSUtility.Color.Red;
        } else {
            return null;
        }
    }



    // 
    // ---------------------------------------------------------------------
    //

    private boolean mInRotate = false;
    private int mSegmentCount = 0;
    private FMSUtility.Color mStartColor;
    private FMSUtility.Color mLastColor;
    private int mExecCallsSinceLastTransition;

    /**
     * implement the RotateCmd need to rotate wheel between 3 and 5 times alert
     * drive station 
     *      - if cannot detect color 
     *      - if wheel is not turning
     */

     /**
      * Check what color we are on, and start the wheel turning
      * Need to test if we need to drive the robot forward at low power to keep engaged with wheel
      * If cannot detect color, alert drive station
      * When > 3 revs, stop wheel
      */
     public void startRotateCmd() {
         mStartColor = getSensorFMSColor(); 
         if (mStartColor == null) {
             mLog.alarm("startRotateCmd:  cannot detect color");
         }
         mLastColor = mStartColor;
         mExecCallsSinceLastTransition = 0;
         mSegmentCount = 0;
         mInRotate = true;
         mCSTalon.set(ColorSysConstants.CS_TALON_ROTATESPEED);
     }


     /**
      * Count the color transitions to see how far we have gone
      * Once we have done 2.5 revolutions, start slowing down
      * If the wheel stops spinning, alert driver
      */
     public void execRotateCmd() {
        FMSUtility.Color currentCol = getSensorFMSColor();
        if (currentCol != mLastColor) {
            mSegmentCount++;
            mLastColor = currentCol;
            mExecCallsSinceLastTransition = 0;
        }
        mExecCallsSinceLastTransition++;
        if (mExecCallsSinceLastTransition > 50 * 3) {
            // haven't had a color transition for 3 seconds - PANIC
            mLog.alarm("execRotateCmd:  ----  Color wheel not rotating  ----");
        }
        if (mSegmentCount >= 13) {
            mCSTalon.set(0);
            mInRotate = false;
        }
        else if (mSegmentCount >= 10) {
            mCSTalon.set(ColorSysConstants.CS_TALON_ROTATESPEED / 2);
        }
     }


     public boolean isRotateFinished() {
         return !mInRotate;
     }


     //
     //  rotate to target  --------------------------------------------------------
     //

     /**
     * POSITION CONTROL: Rotate CONTROL PANEL so a specified color aligns with the
     * sensor for at least five (5) seconds. Once either ALLIANCE reaches Stage 3
     * CAPACITY, FMS relays a specified color (randomly selected by FMS and one (1)
     * of the three (3) colors not currently read by the ALLIANCE’S TRENCH color
     * sensor) to all OPERATOR CONSOLES simultaneously. The specified color may not
     * be the same for both ALLIANCES. See Table 3-4 for details on how the TRENCH
     * light is used during POSTION CONTROL.
     * 
     * Process
     *  read the required color from FMS
     *  calculate color we need under our sensor, number of segments to move
     *  move required segments
     *  hold segment for five seconds = 5 * 50 calls to exec
     */


     private boolean mTargComplete = false;
     private FMSUtility.Color mFMSTargColor;
     private FMSUtility.Color mStartTargColor;


     public void initMoveTarget() {
        mTargComplete = false;
     }


     public void execMoveTarget() {

     }

     public boolean isMoveTargetFinished() {
        return mTargComplete;
     }


    public void setColorWheelUp() {
        mColorWheelSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setColorWheelDown() {
        mColorWheelSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

}