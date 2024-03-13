package frc.team9062.robot.Util.CriticalLED.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.team9062.robot.Util.CriticalLED.CriticalLED;
import frc.team9062.robot.Util.CriticalLED.LEDCommandBase;
import frc.team9062.robot.Util.CriticalLED.CriticalLED.bufferArrayType;

public class strobeColor extends LEDCommandBase {
    int IntervalMs;
    int RGB[];
    Color color;
    int counter = 0;
    double timestamp, timeout;

    public strobeColor(CriticalLED led, int IntervalMs, Color color) {
        super("strobe", led, IntervalMs);
        this.IntervalMs = IntervalMs;
        this.color = color;
    }

    public strobeColor(CriticalLED led, int IntervalMs, Color color, double timeout) {
        super("strobe", led, IntervalMs);
        this.IntervalMs = IntervalMs;
        this.color = color;
        this.timeout = timeout;
    }

    private int[][] BufferRGBArray = new int[this.led.getBufferLength()][3];
    private Color[] BufferColorArray = new Color[this.led.getBufferLength()];

    @Override
    public void init() {
        timestamp = Timer.getFPGATimestamp();

        if(RGB != null) {
            for(int counter = 0; counter < this.led.getBufferLength(); counter++ ) {
                BufferRGBArray[counter][0] = RGB[0];
                BufferRGBArray[counter][1] = RGB[1];
                BufferRGBArray[counter][2] = RGB[2];
            }

            this.led.setLEDBuffer(BufferRGBArray, bufferArrayType.RGB);
        } else {
            for(int counter = 0; counter < this.led.getBufferLength(); counter++ ) {
                BufferColorArray[counter] = color;
            }

            this.led.setLEDBuffer(BufferColorArray);
        }
    }

    @Override
    public void down() {
        this.led.setColourless();
    }

    @Override
    public void up() {
        if(RGB != null) {
            this.led.setLEDBuffer(BufferRGBArray, bufferArrayType.RGB);
        }else{
            this.led.setLEDBuffer(BufferColorArray);
        }
    }

    @Override
    public boolean isFinished() {
        if (timeout != 0) {
            return Timer.getFPGATimestamp() - timestamp > timeout;
        } else {
            return false;
        }
    }
}
