package frc.team9062.robot.Util.CriticalLED;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.wpilibj.util.Color;
import frc.team9062.robot.Util.CriticalLED.Commands.staticColor;

public class LEDManager {
    private CriticalLED led;
    private LEDCommandBase activeCommand = null;
    private boolean init = false;
    private Deque<LEDCommand> process;
    private static LEDManager instance;

    public LEDManager (CriticalLED led) {
        this.led = led;
        process = new ArrayDeque<>();
    }

    public static LEDManager getInstance(CriticalLED led) {
        if(instance == null) {
            instance = new LEDManager(led);
        }

        return instance;
    }

    public LEDCommandBase activeCommand() {
        return activeCommand;
    }

    public void endCurrentCommand() {
        activeCommand.end(false);
        activeCommand = null;
        init = false;
    }
    
    public void runActiveCommand() {
        activeCommand.execute();
    }

    public void setActiveCommand(LEDCommandBase command) {
        if(activeCommand != null) {
            if(!activeCommand.compare(command)){
                endCurrentCommand();
                activeCommand = command;
            }
        } else {
            activeCommand = command;
        }
    }

    public void ManageCommands() {
        if(activeCommand != null) {
            if(init) {
                runActiveCommand();
            } else if(activeCommand.isFinished()) {
                setActiveCommand(new staticColor(led, Color.kGreen));
            } else {
                activeCommand.initialize();
                init = true;
            }
        }else {
            led.setColourless();
        }
    }

    public Runnable getLEDRunnable() {
        return LEDRunnable;
    }

    Runnable LEDRunnable = new Runnable() {
        @Override
        public void run() {
            ManageCommands();
        }
    };

    Thread LEDManagerThread =
        new Thread(
            () -> {
                while (true) {
                    try {
                        ManageCommands();
                        Thread.sleep(20);
                    } catch (Exception e) {
                        e.printStackTrace();
                        endCurrentCommand();
                    }
                }
            }
    );     
}