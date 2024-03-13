package frc.team9062.robot.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CriticalSubsystem extends SubsystemBase implements ICriticalSubsystem{
    Timer timer;
    
    public CriticalSubsystem() {}

    public void start_timer() {
        timer.start();
    }

    public void stop_timer() {
        timer.stop();
    }

    public void reset_timer() {
        timer.reset();
    }

    public Runnable controls = new Runnable() {
        @Override
        public void run() {
            handleStates();
        }
    };
}
