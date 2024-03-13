package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Subsystems.Arm;
import frc.team9062.robot.Subsystems.LEDSubsystem;
import frc.team9062.robot.Subsystems.Rollers;
import frc.team9062.robot.Subsystems.Arm.ARM_STATE;
import frc.team9062.robot.Util.CriticalDeadband;
import frc.team9062.robot.Util.IO;

public class TeleopArm extends Command{
    private Arm arm;
    private Rollers rollers;
    private IO io;
    private CriticalDeadband deadband;
    private LEDSubsystem led;
    private boolean ready = false, isGoal;

    public TeleopArm() {
        this.arm = Arm.getInstance();
        this.rollers = Rollers.getInstance();
        io = IO.getInstance();
        led = LEDSubsystem.getInstance();

        deadband = new CriticalDeadband(0.15);

        addRequirements(arm, rollers);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        if(Math.abs(deadband.applydeadband(() -> io.getOperatorLeftY())) > 0) {
            arm.setArmState(ARM_STATE.MANUAL);
            arm.setArmVelocity(deadband.applydeadband(() -> -io.getOperatorLeftY()) * 2 * Math.PI);
        } else {
            arm.setArmVelocity(0);
        }

        /*
        if (io.getOperatorLeftTrigger(.3)) {
            //arm.setArmState(ARM_STATE.AIM);
            //arm.setArmPosition(Constants.PHYSICAL_CONSTANTS.ARM_INTAKE_POSITION);
            superstructure.setSuperStructureState(STRUCTURE_STATE.INTAKING);
        } else if (io.getOperatorYButton()) {
            arm.setArmState(ARM_STATE.AIM);
            arm.setArmPosition(Constants.PHYSICAL_CONSTANTS.ARM_AMP_POSITION);
        } else if(io.getOperatorBButton()) {
            arm.setArmState(ARM_STATE.AIM);
            arm.setArmPosition(Constants.PHYSICAL_CONSTANTS.ARM_HOLD_POSITION_LOw);
        }else {
            if (arm.getArmState() == ARM_STATE.AIM) {
                arm.setArmPosition(Constants.PHYSICAL_CONSTANTS.ARM_HOLD_POSITION_LOw);
            }
        }
        

        if (io.getOperatorRightTrigger(0.3)) {
            rollers.setIntakeVelocity(2.5);
        } else if(io.getOperatorRightBumper()) {
            rollers.setIntakeVelocity(-2);
        } else {
            rollers.setIntakeVelocity(0);
        }

        if (io.getOperatorYButton()) {
            rollers.setShooterVelocity(30);
        } else if (io.getOperatorBButton()) {
            rollers.setShooterVelocity(40);
        } else {
            rollers.setShooterRaw(0);
        }
        
        */

        /**/ 
    }
}