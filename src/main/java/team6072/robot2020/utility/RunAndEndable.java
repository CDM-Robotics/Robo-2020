package team6072.robot2020.utility;


public interface RunAndEndable extends Runnable{

    /**
     * Makes the thread stop the loop so that the Robot can stop cleanly
     */
    public void end();

}
