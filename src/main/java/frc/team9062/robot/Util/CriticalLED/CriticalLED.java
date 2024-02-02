package frc.team9062.robot.Util.CriticalLED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Robot;

public class CriticalLED extends SubsystemBase {
    private int port;
    private AddressableLED led;
    private AddressableLEDBuffer led_buffer;
    private AddressableLEDBuffer colorlessBuffer;
    private LEDManager ledManager;

    public enum bufferArrayType {
        RGB,
        HSV,
        COL,
    }

    /**
     * Constructs a new instance of CriticalLED from specified PWM port of LED 
     * 
     * @param port PWM port of the Addressable LED [0-9]
     * @param bufferLength Length of the buffer (Number of Pixels on LED)
     */
    public CriticalLED(int port, int bufferLength) {
        this.port = port;

        led = new AddressableLED(port);
        led_buffer = new AddressableLEDBuffer(bufferLength);
        led.setLength(led_buffer.getLength());
        colorlessBuffer = new AddressableLEDBuffer(bufferLength);

        ledManager = new LEDManager(this);

        initColourlessBuffer();
        led.start();
    }

    /** 
     * @return The length of the LEDs Buffer
     */
    public int getBufferLength() {
        return led_buffer.getLength();
    }

    /**
     * Gets the color of a pixel at a specified index
     * 
     * @param index Index of desired pixel
     * @return [Color8Bit] Color of the Pixel at index
     */
    public Color8Bit getLEDPixel(int index) {
        return led_buffer.getLED8Bit(index);
    }

    /**
     * Set the colors of the LED colourless
     */
    public void setColourless() {
        setLEDData(colorlessBuffer);
    }

    /**
     * Set the colors on the LED
     * 
     * @param ledArray [Color] Array containing values for LED
     */
    public void setLEDBuffer(Color[] ledArray) {
        try {
            if(ledArray.length != getBufferLength()) {
                throw new Exception("LENGTH MISMATCH: ARRAY DOES NOT FIT WITH BUFFER LENGTH");
            }            
        } catch (Exception e) {
        }

        for(int counter = 0; counter < getBufferLength(); counter++) {
            led_buffer.setLED(counter, ledArray[counter]);
        }

        led.setData(led_buffer);
    }

    /**
     * [DEPRECIATED] Set the colors on the LED
     * 
     * @param ledArray [Color8Bit] Array containing values for LED
     */
    public void setLEDBuffer(Color8Bit[] ledArray) {
        try {
            if(ledArray.length != getBufferLength()) {
                throw new Exception("LENGTH MISMATCH: ARRAY DOES NOT FIT WITH BUFFER LENGTH");
            }
        } catch (Exception e) {
        }

        for(int counter = 0; counter < getBufferLength(); counter++) {
            led_buffer.setLED(counter, ledArray[counter]);
        }

        setLEDData(led_buffer);
    }

    /**
     * Set the colors on the LED
     * 
     * @param ledArray Array containing values for the buffer
     * @param kType Type of Buffer Array [RGB, HSV]
     */
    public void setLEDBuffer(int[][] ledArray, bufferArrayType kType) {
        try {
            if(ledArray.length != getBufferLength()) {
                throw new Exception("LENGTH MISMATCH: ARRAY DOES NOT FIT WITH BUFFER LENGTH");
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        for(int counter = 0; counter < getBufferLength(); counter++) {
            if(kType == bufferArrayType.RGB) {
                led_buffer.setRGB(counter, ledArray[counter][0], ledArray[counter][1], ledArray[counter][2]);
            }else if( kType == bufferArrayType.HSV ){
                led_buffer.setHSV(counter, ledArray[counter][0], ledArray[counter][1], ledArray[counter][2]);
            }
        }

        setLEDData(led_buffer);
    }

    /**
     * Sets the LED's colours based on the data from the buffer
     * 
     * @param buffer AddressableLEDBuffer containing the values for the led
     */
    public void setLEDData(AddressableLEDBuffer buffer) {
        if(Robot.isReal()){
            led.setData(buffer);
        }
    }

    /**
     * Initializes the colourless buffer
     */
    public void initColourlessBuffer() {
        for(int counter = 0; counter < getBufferLength(); counter++) {
            colorlessBuffer.setLED(counter, Color.kBlack);
        }
    }

    /**
     * Sets the LED Command
     * 
     * @param command LEDCommand to be run on the LEDs
     */
    public void scheduleLEDCommand(LEDCommandBase command) {
        ledManager.setActiveCommand(command);
    }

    public LEDManager getLEDManager() {
        return ledManager;
    }

    /**
     * Starts LED Manager Thread
     */
    public void startLEDManagerThread() {
        ledManager.LEDManagerThread.start();
    }

    /**
     * 
     * @return The PWM Port of the LED
     */
    public int getPort() {
        return port;
    }

    /**
     * Performs sum of colors' RGB values inputed and return a new color. The RGB values do not exceed 255
     * 
     * @param col1 First color to add
     * @param col2 Second color to add
     * @return Sum of colors
     */
    public static Color8Bit add(Color8Bit col1, Color8Bit col2) {
        int r = col1.red + col2.red > 255 ? 255 : col1.red + col2.red;
        int g = col1.green + col2.green > 255 ? 255 : col1.green + col2.green;
        int b = col1.blue + col2.blue > 255 ? 255 : col1.blue + col2.blue;
        
        return new Color8Bit(r, g, b);
    }

    /**
     * Performs subtraction of colors' RGB values inputed and return a new color. The RGB values do not drop below 0
     * 
     * @param col1 First color to subtract
     * @param col2 Second color to subtract
     * @return Difference of colors
     */
    public static Color8Bit subtract(Color8Bit col1, Color8Bit col2) {
        int r = col1.red - col2.red < 0 ? 0 : col1.red - col2.red;
        int g = col1.green - col2.green < 0 ? 0 : col1.green - col2.green;
        int b = col1.blue - col2.blue < 0 ? 0 : col1.blue - col2.blue;
        
        return new Color8Bit(r, g, b);
    }

    /**
     * Performs multiplication of colors' RGB values inputed and return a new color. The RGB values do not exceed 255
     * 
     * @param col1 First color to multiply
     * @param col2 Second color to multiply
     * @return Product of colors
     */
    public static Color8Bit mult(Color8Bit col1, Color8Bit col2) {
        int r = (col1.red * col2.red) % 255;
        int g = (col1.green * col2.green) % 255;
        int b = (col1.blue * col2.blue) % 255;

        return new Color8Bit(r, g, b);
    }
}