
package team6072.robot2020.constants.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class ColorSysConstants {

    public static final int CS_TALON = 1;

    public static final double CS_DRIVEFORWARDPOWER = 0.05;

    public static final boolean CS_TALON_INVERT = false;

    public static final double CS_TALON_ROTATESPEED = 0.3;

    public static final double CS_TALON_SEEKSPEED = 0.1;

    public static final NeutralMode CS_TALON_NEUTRAL_MODE = NeutralMode.Brake;

    public static final int COLOR_DOWN = 0; // intake wheels deployed to get ball
    public static final int COLOR_UP = 1;

}
