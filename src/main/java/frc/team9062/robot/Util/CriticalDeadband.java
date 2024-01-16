package frc.team9062.robot.Util;

import java.util.function.DoubleSupplier;

public class CriticalDeadband {
    private double range = 1, min, max, threshold = 0.1, neutral = 0;

    /**
     * Contructor for instance of CriticalDeadband Class. The default range would be 1
     * 
     * @param threshold The threshold of values of which the output should be 0
     */
    public CriticalDeadband(double threshold) {
        this.threshold = threshold;
    }

     /**
     * Contructor for instance of CriticalDeadband Class
     * 
     * @param threshold The threshold of values of which the output should be 0
     * @param range The range of possible inputs from the axis
     */
    public CriticalDeadband(double threshold, double range) {
        this.threshold = threshold;
        this.range = range;
    }

    /**
     * Changes the threshold of the deadband to a desired value.
     * 
     * @param threshold Throushold you would like to set on the deadband.
     */
    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    /**
     * Changes the range of the deadband
     * 
     * @param range New range of the deadband
     */
    public void setRange(double range) {
        this.range = range;
    }

    /**
     * Set a minimum and maximum value for the deadband. Default minimum is 0 and max is 1. [WARNING] Using this method would override range value
     * 
     * 
     * @param min Minimum value of the deadband
     * @param max Maximum value of the deadband
     * 
     */
    public void setMinMax(double min, double max) {
        this.min = min;
        this.max = max;
    }

    /**
     * Applies deadband to the inputed axis
     * 
     * @param axis DoubleSupplier which the deadband is going to be applied to
     * @return The calculated value of the inputed axis
     */
    public double applydeadband(DoubleSupplier axis) {
        double thresAxis, adjustedRange = range;

        if(Math.abs(axis.getAsDouble()) > threshold) {
            thresAxis = Math.abs(axis.getAsDouble()) - threshold;

            if(!isMaxMin()) {
                return (thresAxis / (adjustedRange - threshold)) * Math.signum(axis.getAsDouble());
            }else {
                adjustedRange = max - min;
                
                return (thresAxis / (adjustedRange - threshold)) * Math.signum(axis.getAsDouble());
            }
        }else {
            return neutral;
        }
    }

    /**
     * Calculates the value that should be outputed from the inputed axis 
     * 
     * @param axis Double which the deadband is going to be applied to
     * @return The calculated value of the inputed axis
     */
    public double applydeadband(double axis) {
        double thresAxis, adjustedRange = range;

        if(Math.abs(axis) > threshold) {
            thresAxis = Math.abs(axis) - threshold;

            if(!isMaxMin()) {
                return (thresAxis / (adjustedRange - threshold)) * Math.signum(axis);
            }else {
                adjustedRange = max - min;
                
                return (thresAxis / (adjustedRange - threshold)) * Math.signum(axis);
            }
        }else{
            return neutral;
        }
    }

    /**
     * Check if there is a Min and Max value in deadband
     * 
     * @return True is there is a Min and Max value and False is there is not
     */
    public boolean isMaxMin() {
        if(max != 0 || min != 0) {
            return true;
        }else{
            return false;
        }
    }
}
