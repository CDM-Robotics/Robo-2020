package team6072.robot2020.logging;

import team6072.robot2020.constants.logging.SuperLogConstants;
import team6072.robot2020.datasources.NavXSource;
import java.util.ArrayList;
import team6072.robot2020.datasources.NavXSource.NavXDataTypes;

/**
 * This doesn't work yet...
 */
public class SuperLogMaster {

    private static SuperLogMaster mSuperLogMaster;
    private SuperLog mNavXYaw;
    private SuperLog mNavXTilt;
    private SuperLog mNavXPitch;

    private ArrayList<SuperLog> mSuperLogs; 

    private static final boolean NAVX_YAW_ON = SuperLogConstants.NAVX_YAW_ON;
    private static final boolean NAVX_TILT_ON = SuperLogConstants.NAVX_TILT_ON;
    private static final boolean NAVX_PITCH_ON = SuperLogConstants.NAVX_PITCH_ON;

    public static SuperLogMaster getInstance(){
        if(mSuperLogMaster == null){
            mSuperLogMaster = new SuperLogMaster();
        }
        return mSuperLogMaster;
    }

    private SuperLogMaster(){
        // NavX Superloggers //
        mNavXYaw = new SuperLog(new NavXSource(NavXDataTypes.YAW), NAVX_YAW_ON);
        mNavXTilt = new SuperLog(new NavXSource(NavXDataTypes.TILT), NAVX_TILT_ON);
        mNavXPitch = new SuperLog(new NavXSource(NavXDataTypes.PITCH), NAVX_PITCH_ON);

        mSuperLogs.add(mNavXYaw);
        mSuperLogs.add(mNavXTilt);
        mSuperLogs.add(mNavXPitch);

        for(SuperLog superLog : mSuperLogs){
            superLog.start();
        }
    }

    public void endAllSuperLoggers(){
        for(SuperLog superLog : mSuperLogs){
            superLog.end();
        }
    }

}