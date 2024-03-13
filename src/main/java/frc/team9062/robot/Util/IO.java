package frc.team9062.robot.Util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class IO {
    private static IO instance;
    private XboxController driver, operator;
    private gamepadConfigs configs = new gamepadConfigs();

    public IO() {
        driver = configs.getDriverController();
        operator = configs.getOperatorController();
    }

    public static IO getInstance() {
        if (instance  == null) {
            instance = new IO();
        }

        return instance;
    }

    public double getDriverLeftX() {
        return driver.getLeftX();
    }

    public double getDriverLeftY() {
        return driver.getLeftY();
    }

    public double getDriverRightX() {
        return driver.getRightX();
    }

    public double getDriverRightY() {
        return driver.getRightY();
    }

    public double getDriverLeftTrigger() {
        return driver.getLeftTriggerAxis();
    }

    public boolean getDriverLeftTrigger(double threshold) {
        return Math.abs(driver.getLeftTriggerAxis()) > threshold;
    }

    public double getDriverRightTrigger() {
        return driver.getRightTriggerAxis();
    }

    public boolean getDriverRightTrigger(double threshold) {
        return Math.abs(driver.getRightTriggerAxis()) > threshold;
    }

    public boolean getDriverXButton() {
        return driver.getXButton();
    }

    public boolean getDriverYButton() {
        return driver.getYButton();
    }

    public boolean getDriverAButton() {
        return driver.getAButton();
    }

    public boolean getDriverBButton() {
        return driver.getBButton();
    }

    public int getDriverPOV() {
        return driver.getPOV();
    }

    public boolean isDriverInput() { // Todo: Complete method
        return false;
    }

    public double getOperatorLeftX() {
        return operator.getLeftX();
    }

    public double getOperatorLeftY() {
        return operator.getLeftY();
    }

    public double getOperatorRightX() {
        return operator.getRightX();
    }

    public double getOperatorRightY() {
        return operator.getRightY();
    }

    public boolean getOperatorLeftBumper() {
        return operator.getLeftBumper();
    }

    public boolean getOperatorRightBumper() {
        return operator.getRightBumper();
    }

    public double getOperatorLeftTrigger() {
        return operator.getLeftTriggerAxis();
    }

    public boolean getOperatorLeftTrigger(double threshold) {
        return Math.abs(operator.getLeftTriggerAxis()) > threshold;
    }

    public double getOperatorRightTrigger() {
        return operator.getRightTriggerAxis();
    }

    public boolean getOperatorRightTrigger(double threshold) {
        return Math.abs(operator.getRightTriggerAxis()) > threshold;
    }

    public boolean getOperatorXButton() {
        return operator.getXButton();
    }

    public boolean getOperatorXButton_() {
        return operator.getXButtonReleased();
    }

    public boolean getOperatorYButton() {
        return operator.getYButton();
    }

    public boolean getOperatorYButton_() {
        return operator.getYButtonReleased();
    }

    public boolean getOperatorAButton() {
        return operator.getAButton();
    }

    public boolean getOperatorBButton() {
        return operator.getBButton();
    }

    public boolean getOperatorBButton_() {
        return operator.getBButtonReleased();
    }

    public int getOperatorPOV() {
        return operator.getPOV();
    }

    public void setDriverRumble(double rumble) {
        driver.setRumble(RumbleType.kBothRumble, rumble);
    }

    public void setDriverRumble(double rumble, RumbleType rumbleType) {
        driver.setRumble(rumbleType, rumble);
    }

    public void setOperatorRumble(double rumble) {
        operator.setRumble(RumbleType.kBothRumble, rumble);
    }

    public void setOperatorRumble(double rumble, RumbleType rumbleType) {
        operator.setRumble(rumbleType, rumble);
    }
}