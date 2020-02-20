package team6072.robot2020.utility;


public abstract class Threaded extends Thread{

    /**
     * Makes the thread stop the loop so that the Robot can stop cleanly
     */
    public abstract void end();


}