package team6072.robot2020.constants.logging;

import team6072.robot2020.utility.logging.LogWrapper.Permission;

public class LoggerConstants {

    // Subsystems //
    public static Permission DRIVESYS_PERMISSION = Permission.ERRORS_ONLY;
    public static Permission ELVSYS_PERMISSION = Permission.ALL;
    public static Permission NAVXSYS_PERMISSION = Permission.ALL;
    public static Permission COLOR_SENSOR_PERMISSION = Permission.ALL;
    public static Permission INTAKE_PERMISSION = Permission.ALL;
    public static Permission CLIMBER_PERMISSION = Permission.ALL;


    // Utility // 
    public static Permission PID_CONTROLLER_PERMISSION = Permission.ERRORS_ONLY;
    public static Permission NETWORK_TABLES_PERMISSION = Permission.ALL;
    public static Permission ROBOT_TRACKER_PERMISSION = Permission.ALL;
    public static Permission WATCH_DOG_PERMISSION = Permission.ALL;

    // Commands //
    public static Permission RELATIVE_DRIVE_CMD = Permission.ERRORS_ONLY;
    public static Permission ARCADE_DRIVE_CMD = Permission.ERRORS_ONLY;
    public static Permission ROTATECS_CMD = Permission.WARNINGS_AND_ERRORS;

    // Control Board //
    public static Permission CONTROL_BOARD_PERMISSION = Permission.ALL;


}