package frc.team9062.robot.Util.CriticalLED;

public interface LEDCommand {

    /**
     * First Cycle in LEDCommand
     */
    default void init() {}

    /**
     * Runs every cycle (in both the up and down cycles)
     */
    default void command() {}

    /**
     * Up cycle in the interval
     */
    default void up() {}

    /**
     * Down cycle in the intervals
     */
    default void down() {}

    /**
     * Code to run at the end of the command
     */
    default void commandEnd() {}
}
