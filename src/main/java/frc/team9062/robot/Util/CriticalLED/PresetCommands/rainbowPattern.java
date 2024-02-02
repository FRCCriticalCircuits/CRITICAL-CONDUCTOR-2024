package frc.team9062.robot.Util.CriticalLED.PresetCommands;

import frc.team9062.robot.Util.CriticalLED.CriticalLED;
import frc.team9062.robot.Util.CriticalLED.LEDCommandBase;
import frc.team9062.robot.Util.CriticalLED.CriticalLED.bufferArrayType;

public class rainbowPattern extends LEDCommandBase {
    private double firstpixelHue;
    private int HSVbufferArray[][];

    public rainbowPattern(CriticalLED led) {
        super("rainbow", led, 100);
    }

    @Override
    public void init() {
        firstpixelHue = 0;
    }

    @Override
    public void command() {
        for(int counter = 0; counter < this.led.getBufferLength(); counter++) {
            double hue = (firstpixelHue + (counter * 180 / this.led.getBufferLength())) % 180;
            HSVbufferArray[counter][0] = (int) hue;
            HSVbufferArray[counter][1] = 255;
            HSVbufferArray[counter][2] = 128;
        }

        firstpixelHue += 3;
        firstpixelHue %= 180;
        this.led.setLEDBuffer(HSVbufferArray, bufferArrayType.HSV);
    }
}
