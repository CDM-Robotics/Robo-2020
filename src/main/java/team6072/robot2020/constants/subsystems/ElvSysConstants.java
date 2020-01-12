package team6072.robot2020.constants.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ElvSysConstants{

    public static final int ELV_TALON = 21;
    public static final int ELV_SLAVE0 = 22;

    public static final boolean ELV_INVERT = true;

    public static final double ELV_CONFIG_OPEN_LOOP_RAMP = .2;

    public static final NeutralMode ELV_NEUTRAL_MODE = NeutralMode.Brake; 

    public static final int ELV_TIME_OUT = 10;


}