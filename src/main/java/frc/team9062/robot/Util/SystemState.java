package frc.team9062.robot.Util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team9062.robot.Constants;

public class SystemState {
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

        Transform2d targetTransform = new Transform2d(
            target.toTranslation2d(), 
            new Rotation2d()
        );

        Pose2d position = new Pose2d(
            new Translation2d(-pose.get().getTranslation().getX(), -pose.get().getTranslation().getY()).rotateBy(pose.get().getRotation()), 
            pose.get().getRotation().rotateBy(pose.get().getRotation())
        );

        Pose2d positionFix = new Pose2d(
            new Translation2d(-pose.get().getX(), -pose.get().getY()), Rotation2d.fromDegrees(180)
        );

        Translation2d positionFixTransform = positionFix.transformBy(targetTransform).getTranslation();

        return new AimData(
            position.getTranslation().getAngle(), 
            positionFixTransform.getNorm()
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
}
