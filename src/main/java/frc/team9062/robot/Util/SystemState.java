package frc.team9062.robot.Util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.Drive.SwerveSubsystem;

public class SystemState extends SubsystemBase {
    private static SystemState instance;
    private VERBOSITY_LEVEL robot_verbosity;

    public SystemState() {
        robot_verbosity = VERBOSITY_LEVEL.LOW;
    }

    public static SystemState getInstance() {
        if (instance == null) {
            instance = new SystemState();
        }

        return instance;
    }

    public void setVerbosity(VERBOSITY_LEVEL verb) {
        robot_verbosity = verb;
    }

    public AimData getShootingData(Supplier<Pose2d> pose) {
        Translation3d target;

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            target = new Translation3d(
                Constants.PHYSICAL_CONSTANTS.FIELD_LENGTH_METERS - Constants.PHYSICAL_CONSTANTS.CENTRE_SPEAKER_OPENING.getX(), 
                Constants.PHYSICAL_CONSTANTS.CENTRE_SPEAKER_OPENING.getY(), 
                Constants.PHYSICAL_CONSTANTS.CENTRE_SPEAKER_OPENING.getZ()
            );
        } else {
            target = Constants.PHYSICAL_CONSTANTS.CENTRE_SPEAKER_OPENING;
        }

        Rotation2d angle;

        Translation2d distances = pose.get().getTranslation().minus(target.toTranslation2d());
        
        angle = Rotation2d.fromDegrees(180 + distances.getAngle().getDegrees());

        return new AimData(
            angle, 
            distances.getNorm()
        );
    }

    public VERBOSITY_LEVEL getVerbosity() {
        return robot_verbosity;
    }

    public enum VERBOSITY_LEVEL {
        HIGH,
        LOW,
        COMP
    }

    public enum ROBOT_STATE {
        DRIVING,
        CLIMBING,
        SHOOTING,
        SHOOT_WHILE_DRIVE
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Distance", getShootingData(SwerveSubsystem.getInstance()::getPoseEstimate).distance);
    }
}
