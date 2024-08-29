package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Subsystems.Drive.SwerveSubsystem;
import frc.team9062.robot.Util.CriticalDeadband;
import frc.team9062.robot.Util.IO;
import frc.team9062.robot.Util.Limelight;
import frc.team9062.robot.Util.SwerveDriveController;
import frc.team9062.robot.Util.SystemState;
import frc.team9062.robot.Util.gamepadConfigs;

public class TeleopDrive extends Command{
    private SwerveSubsystem swerve;
    private Limelight limelight;
    private SwerveDriveController controller;
    private CriticalDeadband deadband = new CriticalDeadband(0.15);
    private IO io;
    private boolean withHeading = false;

    public TeleopDrive() {
        swerve = SwerveSubsystem.getInstance();
        limelight  = Limelight.getInstance("limelight");
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

        if (!io.getDriverLeftTrigger(0.3)) {
            if (withHeading) {
                withHeading = false;
            }

            controller.drive(
                deadband.applydeadband(() -> io.getDriverLeftY()), 
                deadband.applydeadband(() -> io.getDriverLeftX()), 
                deadband.applydeadband(() -> io.getDriverRightX()), 
                true
            );
        } else {
            double heading = SystemState.getInstance().getShootingData(swerve::getPose).heading.getDegrees();

            if(!withHeading) {
                //controller.resetThetaController();
                controller.setThetaController(heading);
                withHeading = true;
            }

            controller.driveWithHeading(
                deadband.applydeadband(() -> io.getDriverLeftY()), 
                deadband.applydeadband(() -> io.getDriverLeftX()),
                heading
            );
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
