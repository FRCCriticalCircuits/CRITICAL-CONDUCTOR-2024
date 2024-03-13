package frc.team9062.robot.Subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.team9062.robot.Util.CriticalSubsystem;
import frc.team9062.robot.Util.CriticalLED.CriticalLED;
import frc.team9062.robot.Util.CriticalLED.LEDCommandBase;
import frc.team9062.robot.Util.CriticalLED.Commands.staticColor;
import frc.team9062.robot.Util.CriticalLED.Commands.strobeColor;

/** IMPLEMENTATION OF LED SUBSYSTEM USING CUSTOM LED Class */
public class LEDSubsystem extends CriticalSubsystem {
    private static LEDSubsystem instance;
    private CriticalLED led;
    private LED_STATE state = LED_STATE.DISABLED;
    private static Map<LED_STATE, LEDCommandBase> led_map = new HashMap<>();

    

    public LEDSubsystem() {
        led = new CriticalLED(0, 16);

        led_map.put(LED_STATE.DISABLED, new staticColor(led, Color.kDarkRed));
        led_map.put(LED_STATE.DEFAULT, new staticColor(led, Color.kGreen));
        led_map.put(LED_STATE.INTAKING, new strobeColor(led, 100, Color.kYellow));
        led_map.put(LED_STATE.SHOOTING, new staticColor(led, Color.kWhite));
        led_map.put(LED_STATE.CONFIRM, new strobeColor(led, 40, Color.kGreen, 1));
        led_map.put(LED_STATE.ALIGNING, new strobeColor(led, 100, Color.kWhite));

        led.startLEDManagerThread();
    }

    public static LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }

        return instance;
    }

    public void setLED(LED_STATE state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            state = LED_STATE.DISABLED;
        }
        
        switch (state) {
            case CONFIRM -> {

            }
            default -> {
                led.getLEDManager().setActiveCommand(
                    led_map.get(state)
                );
            }
        }
    }

    public enum LED_STATE {
        DEFAULT,
        ALLIANCE,
        DISABLED,
        INTAKING,
        ALIGNING,
        SHOOTING,
        CLIMBING,
        CONFIRM
    }
}
