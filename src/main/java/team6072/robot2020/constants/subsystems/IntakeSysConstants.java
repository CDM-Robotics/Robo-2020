/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package team6072.robot2020.constants.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Add your docs here.
 */
public class IntakeSysConstants {

    public static final int INTAKE_TALON = 2;
    public static final boolean INTAKE_TALON_INVERT = false;
    public static final double INTAKE_TALON_ROTATESPEED = 0.3;
    public static final NeutralMode INTAKE_TALON_NEUTRAL_MODE = NeutralMode.Coast;

    public static final int BELT_TALON = 3;
    public static final boolean BELT_TALON_INVERT = false;
    public static final double BELT_TALON_ROTATESPEED = 0.3;
    public static final NeutralMode BELT_TALON_NEUTRAL_MODE = NeutralMode.Coast;

    public static final int CAN_TALON = 4;
    public static final boolean CAN_TALON_INVERT = false;
    public static final double CAN_TALON_ROTATESPEED = 0.3;
    public static final NeutralMode CAN_TALON_NEUTRAL_MODE = NeutralMode.Coast;

    public static final int INTAKE_DOWN = 0;        // intake wheels deployed to get ball
    public static final int INTAKE_UP = 1;

}
