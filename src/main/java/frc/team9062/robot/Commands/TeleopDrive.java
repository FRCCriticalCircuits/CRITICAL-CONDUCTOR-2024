package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Subsystems.SwerveSubsystem;
import frc.team9062.robot.Util.CriticalDeadband;
import frc.team9062.robot.Util.SwerveDriveController;
import frc.team9062.robot.Util.gamepadConfigs;

public class TeleopDrive extends Command{
    private SwerveSubsystem swerve;
    private gamepadConfigs controls;
    private SwerveDriveController controller;
    private CriticalDeadband deadband = new CriticalDeadband(0.1);

    public TeleopDrive() {
        swerve = SwerveSubsystem.getInstance();
        controls = gamepadConfigs.getInstance();
        controller = new SwerveDriveController(false);

        addRequirements(swerve);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        controller.drive(
            deadband.applydeadband(() -> controls.getDriverController().getLeftY()), 
            deadband.applydeadband(() -> controls.getDriverController().getLeftX()), 
            deadband.applydeadband(() -> controls.getDriverController().getRightX()), 
            true
        );
    }

    @Override
    public void end(boolean interupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
