package team6072.robot2020.constants.logging;

import team6072.robot2020.logging.LogWrapper.Permission;

public class LoggerConstants {

    // Subsystems //
    public static Permission DRIVESYS_PERMISSION = Permission.ALL;
    public static Permission ELVSYS_PERMISSION = Permission.ALL;
    public static Permission NAVXSYS_PERMISSION = Permission.ALL;
    public static Permission COLOR_SENSOR_PERMISSION = Permission.ALL;


    // PID // 
    public static Permission PID_CONTROLLER_PERMISSION = Permission.ALL;

    // Commands //
    public static Permission RELATIVE_DRIVE_CMD = Permission.ERRORS_ONLY;
    public static Permission ARCADE_DRIVE_CMD = Permission.ERRORS_ONLY;

    // Control Board //
    public static Permission CONTROL_BOARD_PERMISSION = Permission.ALL;

    // Network Tables Thread //
    public static Permission NETWORK_TABLES_PERMISSION = Permission.ALL;

}