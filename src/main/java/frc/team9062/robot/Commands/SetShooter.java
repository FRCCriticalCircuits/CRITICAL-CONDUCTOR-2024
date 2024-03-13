package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Subsystems.Rollers;

public class SetShooter extends Command {
    private Rollers rollers;
    private double velocity;

    public SetShooter(double velocity) {
        rollers = Rollers.getInstance();

        this.velocity = velocity;
    }

    @Override
    public void execute() {
        rollers.setShooterVelocity(velocity);
    }

    @Override
    public void end(boolean interupted) {
        rollers.setShooterVelocity(0);
    }
}
