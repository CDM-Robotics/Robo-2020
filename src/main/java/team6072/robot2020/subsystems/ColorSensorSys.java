package team6072.robot2020.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.I2C.Port;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.FMSUtility;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorSys {

    private static ColorSensorSys mColorSensorSys;

    private Color mBlue;
    private Color mRed;
    private Color mYellow;
    private Color mGreen;

    private LogWrapper mLog;
    private ColorSensorV3 mSensor;
    private ColorMatch mColorMatch;

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

}