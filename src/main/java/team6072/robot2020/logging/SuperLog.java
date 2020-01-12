package team6072.robot2020.logging;

import team6072.robot2020.datasources.DataSourceBase;

/**
 * This doesn't work yet....
 */
public class SuperLog extends Thread{

    private DataSourceBase mDataSourceBase;

    public SuperLog(DataSourceBase dataSourceBase, boolean on) {
        mDataSourceBase = dataSourceBase;
    }

    public void run(){
        // send data to excel sheet
        
    }
    

    public void end(){
        // ends the thread
    }

}