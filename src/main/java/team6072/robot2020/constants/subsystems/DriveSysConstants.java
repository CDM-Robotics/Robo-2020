package team6072.robot2020.constants.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveSysConstants {

    // Motor percent Numbers //
    public static final double BASE_PERCENT_OUT = 0.0;

    // Motor Numbers //
    public static final int LEFT_TALON_MASTER = 01;
    public static final int LEFT_TALON_SLAVE0 = 02;
    public static final int LEFT_TALON_SLAVE1 = 03;
    public static final int RIGHT_TALON_MASTER = 11;
    public static final int RIGHT_TALON_SLAVE0 = 12;
    public static final int RIGHT_TALON_SLAVE1 = 13;

    public static final boolean LEFT_TALON_MASTER_SENSOR_PHASE = true;
    public static final boolean LEFT_TALON_SLAVE0_SENSOR_PHASE = true;
    public static final boolean LEFT_TALON_SLAVE1_SENSOR_PHASE = true;
    public static final boolean RIGHT_TALON_MASTER_SENSOR_PHASE = true;
    public static final boolean RIGHT_TALON_SLAVE0_SENSOR_PHASE = true;
    public static final boolean RIGHT_TALON_SLAVE1_SENSOR_PHASE = true;

    // Motor settings //
    public static final boolean DRIVE_LEFT_INVERT = true;
    public static final boolean DRIVE_RIGHT_INVERT = true;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
    public static final double DRIVE_CONFIG_OPEN_LOOP_RAMP = .1;
    public static final int DRIVE_TIME_OUT = 10;

    // Swerve Drive // still must be set
    public static final double RELATIVE_P = (1.0 / 75.0);
    public static final double RELATIVE_D = 0.0;
    public static final double RELATIVE_I = 0.0;
    public static final double RELATIVE_F = 0.0;
    public static final double RELATIVE_UPPER_DEADBAND = 0.23;
    public static final double RELATIVE_LOWER_DEADBAND = -0.23;
    public static final double RELATIVE_YAW_TOLERANCE = 0.2;

}