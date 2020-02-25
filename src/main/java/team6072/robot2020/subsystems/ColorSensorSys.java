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
    }



    public int getDistance() {
        return mSensor.getProximity();
    }



    public FMSUtility.Color matchColor() {
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

    public Color getColor() {
        return mSensor.getColor();
    }


    // 
    // ---------------------------------------------------------------------
    //

    private boolean mInRotate = false;
    private int mSegmentCount = 0;
    private FMSUtility.Color startColor;

    /**
     * implement the RotateCmd need to rotate wheel between 3 and 5 times alert
     * drive station 
     *      - if cannot detect color 
     *      - if wheel is not turning
     */

     public void startRotateCmd() {
         startColor = matchColor(); 
         if (startColor == null) {
             mLog.alarm("startRotateCmd:  cannot detect color");
         }

     }



}