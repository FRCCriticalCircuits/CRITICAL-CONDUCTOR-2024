package frc.team9062.robot.Util.CriticalLED;

/**
 * Contains possible state of LED.
 */
public class LEDState {
    String stateName;
    LEDCommand command;
    int timeout;

    /**
     * 
     * @param name Name of LEDState
     * @param command LED command
     * @param priority Priority code for LED state [0-2] (The lower the number the higher the priority)
     * @param timeout Command timeout
     */
    public LEDState(String name, LEDCommand command, int priority, int timeout) {
        this.command = command;
        this.timeout = timeout;
        stateName = name; 
    }

    public String getName() {
        return stateName;
    }

    public LEDCommand getCommand() {
        return command;
    }

    public int getTimeout() {
        return timeout;
    }
}
