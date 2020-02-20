package team6072.robot2020.utility;

import edu.wpi.first.wpilibj.Joystick;

public class LogitechJoystick extends Joystick {

    public LogitechJoystick(int portNumber) {
        super(portNumber);
    }

    /**
     * This function gets the Y from the josytick and multplies it by -1 so that
     * pushing the joystick forward will result in positive numbers rather than the
     * usual negative numbers.
     * 
     * @return
     */
    public double getInvertedY() {
        return -getY();
    }

}