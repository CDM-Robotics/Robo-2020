package team6072.robot2020.utility.nt;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.constants.logging.LoggerConstants;

public abstract class NTOnChangeListener{

    private NetworkTableEntry mNetworkTableEntry;
    private NetworkTableValue mPriorValue;
    private LogWrapper mLog;

    public NTOnChangeListener(NetworkTableEntry networkTableEntry){
        mNetworkTableEntry = networkTableEntry;
        mPriorValue = mNetworkTableEntry.getValue();
        mLog = new LogWrapper(FileType.UTILITY, "NT Listener", LoggerConstants.NETWORK_TABLES_PERMISSION);
    }

    public void checkState(){
        NetworkTableValue currentValue = mNetworkTableEntry.getValue();
        // mLog.periodicPrint("Current Value double: " + currentValue.isDouble(), 20000);
        // test if the value is a double
        if(currentValue.isDouble()){
            if(currentValue.getDouble() != mPriorValue.getDouble()){
                mPriorValue = currentValue;
                execute();
            }
        }
        // test if the value is a boolean
        if(currentValue.isBoolean()){
            if(currentValue.getBoolean() != mPriorValue.getBoolean()){
                mPriorValue = currentValue;
                execute();
            }
        }

    }

    public NetworkTableEntry getEntry(){
        return mNetworkTableEntry;
    }

    public abstract void execute();

}

