package frc.team9062.robot.Util;

import edu.wpi.first.wpilibj.XboxController;

public class IO {
    private XboxController driver, operator;
    private gamepadConfigs configs;

    public IO() {
        driver = configs.getDriverController();
    }
}
