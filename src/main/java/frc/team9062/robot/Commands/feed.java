package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Subsystems.Rollers;
import frc.team9062.robot.Subsystems.Rollers.INTAKE_STATE;

public class feed extends Command {
    private Rollers rollers;
    
    public feed() {
        rollers = Rollers.getInstance();
    }

    @Override
    public void execute() {
        rollers.setIntakeState(INTAKE_STATE.FEEDING);
    }

    @Override
    public void end(boolean interupted) {
        rollers.setIntakeState(INTAKE_STATE.IDLE);
    }

    @Override
    public boolean isFinished() {
        return !rollers.isWithGamePiece();
    }
}
