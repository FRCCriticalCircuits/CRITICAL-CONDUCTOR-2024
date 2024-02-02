package frc.team9062.robot.Util.CriticalLED.PresetCommands;

import frc.team9062.robot.Util.CriticalLED.CriticalLED;
import frc.team9062.robot.Util.CriticalLED.LEDCommandBase;
import frc.team9062.robot.Util.CriticalLED.CriticalLED.bufferArrayType;

public class pulseColor extends LEDCommandBase{
    private int HSV[];
    private boolean falling = true;
    
    private int[][] HSVBufferArray;
    
    public pulseColor(CriticalLED led, int HSV[]) {
        super("pulse", led, 500);
        this.HSV = HSV;

        HSVBufferArray = new int[led.getBufferLength()][3];
    }

    @Override
    public void init() {
        for(int index = 0; index < led.getBufferLength(); index++) {
            HSVBufferArray[index][0] = HSV[0];
            HSVBufferArray[index][1] = HSV[1];
            HSVBufferArray[index][2] = HSV[2];
        }

        this.led.setLEDBuffer(HSVBufferArray, bufferArrayType.HSV);
    }

    @Override
    public void command() {
        if(HSVBufferArray[(int) (Math.floor(Math.random() * 100) % 60)][2] > 0) {
            if(HSVBufferArray[(int) (Math.floor(Math.random() * 100) % 60)][2] > 250 && !falling) {
                falling = true;
            }else {
                for(int index = 0; index < led.getBufferLength(); index++) {
                    HSVBufferArray[index][2] -= 15;
                }
            }
        }else {
            if(falling) {
                falling = false;
            }else {
                for(int index = 0; index < led.getBufferLength(); index++) {
                    HSVBufferArray[index][2] += 15;
                }
            }
        }

        this.led.setLEDBuffer(HSVBufferArray, bufferArrayType.HSV);
    }
}