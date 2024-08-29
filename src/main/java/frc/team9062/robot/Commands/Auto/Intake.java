package frc.team9062.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Subsystems.Arm;
import frc.team9062.robot.Subsystems.Rollers;
import frc.team9062.robot.Subsystems.Arm.ARM_STATE;
import frc.team9062.robot.Subsystems.Rollers.INTAKE_STATE;

public class Intake extends Command {
    Arm arm;
    Rollers rollers;

    public Intake() {
        arm = Arm.getInstance();
        rollers = Rollers.getInstance();

        addRequirements(arm, rollers);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.setArmState(ARM_STATE.INTAKE);
        rollers.setIntakeState(INTAKE_STATE.INTAKING);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmState(ARM_STATE.LOW);
        rollers.setIntakeState(INTAKE_STATE.IDLE);
    }

    @Override
    public boolean isFinished() {
        return rollers.isWithGamePiece();
    }
}