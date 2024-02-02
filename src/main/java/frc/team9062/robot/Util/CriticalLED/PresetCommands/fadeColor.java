package frc.team9062.robot.Util.CriticalLED.PresetCommands;

import edu.wpi.first.wpilibj.util.Color;
import frc.team9062.robot.Util.CriticalLED.CriticalLED;
import frc.team9062.robot.Util.CriticalLED.LEDCommandBase;
import frc.team9062.robot.Util.CriticalLED.CriticalLED.bufferArrayType;

public class fadeColor extends LEDCommandBase{
    private Color color;
    private double[] RGB;
    private double hue;
    private int[][] BufferArray;
    private double value = 255;

    public fadeColor(CriticalLED led, Color color) {
        super("fade", led);
        this.color = color;
    }

    public fadeColor(CriticalLED led, double[] RGB) {
        super("fade", led);
        this.RGB = RGB;
    }

    @Override
    public void init() {
        BufferArray = new int[this.led.getBufferLength()][3];

        //Regularize RGB Values
        double r, g, b;
        if(color != null) {
            r = color.red;
            g = color.green;
            b = color.blue;
        } else {
            r = RGB[0];
            g = RGB[1];
            b = RGB[2];
        }

        double maxColor = Math.max(r, Math.max(g, b));
        double minColor = Math.min(r, Math.min(g, b));
        double difference = maxColor - minColor;

        if(maxColor == minColor) {
            hue = 0;
        
        } else if(maxColor == r) {
            hue = (60 * ((g - b) / difference) + 360) % 360;
        
        } else if(maxColor == g) {
            hue = (60 * ((b - r) / difference) + 120) % 360;
        
        } else if(maxColor == b) {
            hue = (60 * ((r - g) / difference) + 240) % 360;
        
        }
    }

    @Override
    public void command() {
        for(int counter = 0; counter < this.led.getBufferLength(); counter++) {
            BufferArray[counter][0] = (int) (hue);
            BufferArray[counter][1] = 255;
            BufferArray[counter][2] = (int) value;
        }

        this.led.setLEDBuffer(BufferArray, bufferArrayType.HSV);

        value -= 1;
        if(value < 0) {
            value = 255;
        }
    }
}