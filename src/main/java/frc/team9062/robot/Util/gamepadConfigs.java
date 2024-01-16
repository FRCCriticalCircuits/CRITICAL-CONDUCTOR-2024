package frc.team9062.robot.Util;

import edu.wpi.first.wpilibj.XboxController;
import frc.team9062.robot.Constants;

public class gamepadConfigs {
    private static gamepadConfigs instance;
    private XboxController driver, operator;

    public gamepadConfigs() {
        driver = new XboxController(Constants.DEVICE_IDs.GAMEPAD_DRIVER);
        operator = new XboxController(Constants.DEVICE_IDs.GAMEPAD_OPERATOR);
    }

    public XboxController getDriverController() {
        return driver;
    }

    public XboxController getOperatorController() {
        return operator;
    }
 
    public static gamepadConfigs getInstance() {
        if (instance == null) {
            instance = new gamepadConfigs();
        }

        return instance;
    }

    public class TeleopBinds {

    }

    public class Triggers {

    }
}
