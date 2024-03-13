package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Subsystems.Arm;
import frc.team9062.robot.Subsystems.Arm.ARM_STATE;
import frc.team9062.robot.Util.IO;

public class manualArm extends Command {
    public Arm arm;

    public manualArm() {
        arm = Arm.getInstance();

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmState(ARM_STATE.MANUAL);
    }

    @Override
    public void execute() {
        arm.setArmVelocity(-IO.getInstance().getOperatorLeftY() * Math.PI * 2);
    }

    @Override
    public void end(boolean interupted) {
        arm.setArmVelocity(0);
    }

     @Override
    public boolean isFinished() {
        return false;
    }
}
