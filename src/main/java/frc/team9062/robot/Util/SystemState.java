package frc.team9062.robot.Util;

public class SystemState {
    private static SystemState instance;
    private VERBOSITY_LEVEL robot_verbosity;

    public SystemState() {
        robot_verbosity = VERBOSITY_LEVEL.LOW;
    }

    public static SystemState getInstance() {
        if (instance == null) {
            instance = new SystemState();
        }

        return instance;
    }

    public void setVerbosity(VERBOSITY_LEVEL verb) {
        robot_verbosity = verb;
    }

    public VERBOSITY_LEVEL getVerbosity() {
        return robot_verbosity;
    }

    public enum VERBOSITY_LEVEL {
        HIGH,
        LOW,
        COMP
    }

    public enum ROBOT_STATE {
        DRIVING,
        CLIMBING,
        SHOOTING,
        SHOOT_WHILE_DRIVE
    }
}
