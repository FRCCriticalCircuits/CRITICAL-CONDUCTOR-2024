package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Subsystems.SwerveSubsystem;
import frc.team9062.robot.Util.CriticalDeadband;
import frc.team9062.robot.Util.IO;
import frc.team9062.robot.Util.SwerveDriveController;
import frc.team9062.robot.Util.gamepadConfigs;

public class TeleopDrive extends Command{
    private SwerveSubsystem swerve;
    private SwerveDriveController controller;
    private CriticalDeadband deadband = new CriticalDeadband(0.15);
    private IO io;
    private boolean headingMode = false;

    public TeleopDrive() {
        swerve = SwerveSubsystem.getInstance();
        gamepadConfigs.getInstance();
        controller = new SwerveDriveController(true);
        io = IO.getInstance();

        addRequirements(swerve);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (io.getDriverYButton()) {
            swerve.zeroGyro();
        };

        if (!headingMode) {
            controller.drive(
                deadband.applydeadband(() -> io.getDriverLeftY()), 
                deadband.applydeadband(() -> io.getDriverLeftX()), 
                deadband.applydeadband(() -> io.getDriverRightX()), 
                true
            );
        } else {
            
        }
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
