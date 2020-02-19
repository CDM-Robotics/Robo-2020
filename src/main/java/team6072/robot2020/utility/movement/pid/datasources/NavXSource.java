package team6072.robot2020.utility.movement.pid.datasources;

import team6072.robot2020.subsystems.NavXSys;

/**
 * THis is a source for the navx that can be created to look at a specific value from the navx and return data accordingly
 */
public class NavXSource extends DataSourceBase {

    private NavXSys mNavxSys;
    private NavXDataTypes mNavXDataTypes;

    public enum NavXDataTypes {
        YAW, TILT, PITCH, TOTAL_YAW;
    }

    public NavXSource(NavXDataTypes navXDataType) {
        mNavxSys = NavXSys.getInstance();
        mNavXDataTypes = navXDataType;
    }

    public double getData() {
        if (mNavXDataTypes == NavXDataTypes.YAW) {
            return mNavxSys.getYaw();
        } else if (mNavXDataTypes == NavXDataTypes.PITCH) {
            return mNavxSys.getPitch();
        } else if (mNavXDataTypes == NavXDataTypes.TILT) {
            return mNavxSys.getRoll();
        } else if (mNavXDataTypes == NavXDataTypes.TOTAL_YAW) {
            return mNavxSys.getAccumulatedYaw();
        }else {
            return 0.0;
        }

    }

}