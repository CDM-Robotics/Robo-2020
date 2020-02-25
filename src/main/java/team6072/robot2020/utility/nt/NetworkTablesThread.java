package team6072.robot2020.utility.nt;

import java.util.ArrayList;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import team6072.robot2020.utility.logging.LogWrapper;
import team6072.robot2020.constants.logging.LoggerConstants;
import team6072.robot2020.utility.logging.LogWrapper.FileType;
import team6072.robot2020.utility.RunAndEndable;

/**
 * Extends thread, use .start() to start the thread use .end() to end the thread
 */
public class NetworkTablesThread implements RunAndEndable {

  // standard Variables //
  private static NetworkTablesThread mNetworkTablesController;
  private LogWrapper mLog;
  public boolean mCanRun = true;

  // Network Tables Variables //
  private NetworkTableInstance ntinst;
  private NetworkTable mVisionTable;
  private ArrayList<NTOnChangeListener> mListeners;

  private NetworkTableEntry mDistanceEntry;

  public static NetworkTablesThread getInstance() {
    if (mNetworkTablesController == null) {
      mNetworkTablesController = new NetworkTablesThread();
    }
    return mNetworkTablesController;
  }

  private NetworkTablesThread() {
    mLog = new LogWrapper(FileType.UTILITY, "Network Tables Thread", LoggerConstants.NETWORK_TABLES_PERMISSION);

    // initializing Network Tables //
    ntinst = NetworkTableInstance.getDefault();

    mLog.print("Setting up NetworkTables client for team " + 6072);
    ntinst.startClientTeam(6072); // This starts Network tables on the raspberry Pi so that it can connect to the
                                  // Roborio and radio

    mListeners = new ArrayList<NTOnChangeListener>(); // its an array...
    mVisionTable = ntinst.getTable("Vision Table"); // initializing new Vision table on Network tables

    // initializing Entries //
    mDistanceEntry = mVisionTable.getEntry("distance");

    // setting the entry //
    mDistanceEntry.setBoolean(false);

    // initializing listeners tp the mListeners Array //
    NTOnChangeListener mDistanceListener = new NTOnChangeListener(mDistanceEntry) {
      @Override
      public void execute() {
        // TODO Auto-generated method stub
        mLog.alarm("THE VALUE CHANGED TO " + this.getEntry().getBoolean(false));
      }
    };

    // add listeners to array //
    mLog.alarm("Adding Listener");
    mListeners.add(mDistanceListener);
  }

  public void run() {
    // mLog.print("Starting Network Tables Listener Thread");
    mCanRun = true;
    while (mCanRun) {
      // mLog.periodicPrint("Network Tables THread", 20000);
      for (int i = 0; i < mListeners.size(); i++) {
        mListeners.get(i).checkState();
      }
      try {
        Thread.sleep(20);
      } catch (InterruptedException err) {
        mLog.error(err.toString());
      }
    }
  }

  public void end() {
    mCanRun = false;
  }

  /**
   * This function returns a CvSource that can be used to put frames of video onto
   * Network Tables Use cvSource.putFrame(Mat m); to put the frame onto Network
   * tables
   * 
   * @param name
   * @return
   */
  public CvSource getNewCvSource(String name) {
    return CameraServer.getInstance().putVideo(name, 160, 120);
  }

}