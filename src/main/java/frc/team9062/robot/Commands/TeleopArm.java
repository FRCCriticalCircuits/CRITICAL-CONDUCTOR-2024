package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.Arm;
import frc.team9062.robot.Util.CriticalDeadband;
import frc.team9062.robot.Util.IO;

public class TeleopArm extends Command{
    private Arm arm;
    private IO io;
    private CriticalDeadband deadband;

    public TeleopArm() {
        this.arm = Arm.getInstance();
        io = IO.getInstance();

        deadband = new CriticalDeadband(0.15);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        //arm.useLastSetpoint(true);
    }

    @Override
    public void execute() {
        arm.setRaw(deadband.applydeadband(() -> io.getOperatorLeftY()));

        if (io.getOperatorLeftTrigger(0.3)) {
            arm.setPositionSmartMotion(Constants.PHYSICAL_CONSTANTS.ARM_INTAKE_POSITION);
        } else if(io.getOperatorAButton()){
            arm.setPositionSmartMotion(Constants.PHYSICAL_CONSTANTS.ARM_HOLD_POSITION);
        }
    }
}
