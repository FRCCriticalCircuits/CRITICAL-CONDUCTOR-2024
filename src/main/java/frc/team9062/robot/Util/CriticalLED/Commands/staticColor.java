package frc.team9062.robot.Util.CriticalLED.Commands;

import edu.wpi.first.wpilibj.util.Color;
import frc.team9062.robot.Util.CriticalLED.CriticalLED;
import frc.team9062.robot.Util.CriticalLED.LEDCommandBase;
import frc.team9062.robot.Util.CriticalLED.CriticalLED.bufferArrayType;

public class staticColor extends LEDCommandBase {
    private Color color;
    private int[] RGB;

    public staticColor(CriticalLED led, Color color) {
        super("static", led);
        this.color = color;
    }

    public staticColor(CriticalLED led, int[] RGB) {
        super("static", led);
        this.RGB = RGB;
    }
    
    @Override
    public void init() {
        int RGBArray[][] = new int[led.getBufferLength()][3];
        Color[] colorArray = new Color[led.getBufferLength()];

        if(RGB != null) {
            for( int counter = 0; counter < led.getBufferLength(); counter++ ) {
                RGBArray[counter][0] = RGB[0];
                RGBArray[counter][1] = RGB[1];
                RGBArray[counter][2] = RGB[2];
            }
            led.setLEDBuffer(RGBArray, bufferArrayType.RGB);
        } else {
            for( int counter = 0; counter < led.getBufferLength(); counter++ ) {
                colorArray[counter] = color;
            }

            led.setLEDBuffer(colorArray);
        }
    }

    @Override
    public void command() {}
    
    @Override
    public void commandEnd() {}
}
