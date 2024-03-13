package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Subsystems.Arm;
import frc.team9062.robot.Subsystems.Rollers;

public class runSuperStructureState extends Command {
    private Arm arm;
    private Rollers rollers;
    private double arm_position, intake_vel, shooter_vel;

    public runSuperStructureState(double arm_position, double intake_vel, double shooter_vel) {
        this.arm_position = arm_position;
        this.intake_vel = intake_vel;
        this.shooter_vel = shooter_vel;

        arm = Arm.getInstance();
        rollers = Rollers.getInstance();

        addRequirements(arm, rollers);
    }

    @Override
    public void execute() {
        arm.setArmPosition(arm_position);
        rollers.setIntakeVelocity(intake_vel);
        rollers.setShooterVelocity(shooter_vel);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
