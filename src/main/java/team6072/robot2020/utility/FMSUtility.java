package team6072.robot2020.utility;

import edu.wpi.first.wpilibj.DriverStation;

public class FMSUtility {

    public static FMSUtility.Color getColor() {
        String gameData;

        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
            case 'B':
                // Blue case code
                return Color.Blue;
            case 'G':
                // Green case code
                return Color.Green;
            case 'R':
                // Red case code
                return Color.Red;
            case 'Y':
                // Yellow case code
                return Color.Yellow;
            default:
                // This is corrupt data
                return null;
            }
        } else {
            // Code for no data received yet
            return null;
        }
    }

    public enum Color {
        Green, Blue, Red, Yellow;
    }

}