package frc.team9062.robot.Util.CriticalLED;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class LEDCommandBase extends Command implements LEDCommand{
    public CriticalLED led;
    private int IntervalMs = 200, counter = 0;
    private String commandname;

    public LEDCommandBase(String name, CriticalLED led) {
        commandname = name;
        this.led = led;
    }

    public LEDCommandBase(String name, CriticalLED led, int IntervalMs) {
        commandname = name;
        this.led = led;
        this.IntervalMs = IntervalMs;
    }

    @Override
    public void initialize() {
        this.init();
        
        if(IntervalMs == 0) {
            IntervalMs = 160;
        } else if(IntervalMs < 20) {
            IntervalMs = 20;
        } else if((IntervalMs % 20) != 0) {
            if(IntervalMs % 20 > 10) {
                IntervalMs += IntervalMs % 20;
            }else {
                IntervalMs -= IntervalMs % 20;
            }
        }
    }

    @Override
    public void execute() {
        counter++;

        if((IntervalMs / 20) == counter) {
            this.down();
            command();
        }else if((IntervalMs / 10) == counter) {
            this.up();
            command();

            counter = 0;
        }
    }

    @Override
    public void end(boolean interrupt) {
        this.commandEnd();
    }

    /**
     * @return  Name of the command
     */
    public String getCommandName() {
        return commandname;
    }

    /**
     * Gets the time between intervals in ms.
     * 
     * @return Interval between up and down cycle
     * 
     */
    public int getIntervalMS() {
        return IntervalMs;
    }

    /**
     * Compare two command to find differences between commands.
     * 
     * @param command Command to be compared with.
     * 
     * @return Boolean indicating if command are the same, 
     * if true commands are the same and if false commands are not the same.
     * 
     */
    public boolean compare(LEDCommandBase command) {
        if(commandname == command.getCommandName() && IntervalMs == command.getIntervalMS()) {
            return true;
        } else {
            return false;
        }
    }
}