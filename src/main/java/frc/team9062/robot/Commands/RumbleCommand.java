package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * RumbleCommand to set vibration to controllers for set amount of time
 */
public class RumbleCommand extends Command{
    XboxController[] controllers;
    double timestamp, strength, timeout;

    /**
     * Contructor for RumbleCommand. Defaul Strength will be 0.3
     * 
     * @param timeout Amount of time for controller to vibrate
     * @param controller Controllers to be used in this command
     */
    public RumbleCommand(double timeout, XboxController... controller) {
        this.controllers = controller;
        this.timeout = timeout;
    }

    /**
     * Contructor for RumbleCommand.
     * 
     * @param strength Strength of Rumble
     * @param timeout Amount of time for controller to vibrate
     * @param controller Controllers to be used in this command
     */
    public RumbleCommand(double strength, double timeout,XboxController... controller) {
        this.controllers = controller;
        this.strength = strength;
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        timestamp = Timer.getFPGATimestamp();
        for(XboxController controller : controllers) {
            controller.setRumble(RumbleType.kBothRumble, strength != 0 ? strength : 0.3);
        }
    }

    @Override
    public void end(boolean interupted) {
        for(XboxController controller : controllers) {
            controller.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - timestamp > timeout) {
            return true;
        } else {
            return false;
        }
    }

    
}