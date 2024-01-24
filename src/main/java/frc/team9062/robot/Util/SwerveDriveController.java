package frc.team9062.robot.Util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.SwerveSubsystem;

public class SwerveDriveController {
    private SwerveSubsystem swerve;
    private SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private ProfiledPIDController thetaController;
    private boolean enableSlewRateLimiters;

    public SwerveDriveController(boolean enableSlewRateLimiters) {
        swerve = SwerveSubsystem.getInstance();
        this.enableSlewRateLimiters = enableSlewRateLimiters;
        
        if (enableSlewRateLimiters) {
            xLimiter = new SlewRateLimiter(8);
            yLimiter = new SlewRateLimiter(8);
            thetaLimiter = new SlewRateLimiter(8);
        }

        thetaController = new ProfiledPIDController(
            Constants.TUNED_CONSTANTS.THETA_PID_P, 
            Constants.TUNED_CONSTANTS.THETA_PID_I, 
            Constants.TUNED_CONSTANTS.THETA_PID_D, 
            new Constraints(
                Constants.TUNED_CONSTANTS.THETA_MAX_DEG_S, 
                Constants.TUNED_CONSTANTS.THETA_MAX_DEG_S2
            )
        );

        thetaController.enableContinuousInput(-180, 180);
    }

    public void drive(double x, double y, double theta, boolean isFieldRelative) {
        double[] inputs = convertInputs(x, y, theta);

        if (enableSlewRateLimiters) {
            applyLimiters(inputs);
        }

        ChassisSpeeds speeds;

        if (isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                inputs[0],
                inputs[1],
                inputs[2], 
                swerve.getAngle()
            );
        } else {
            speeds = new ChassisSpeeds(inputs[0], inputs[1], inputs[2]);
        }

        speeds = correctDynamics(speeds);

        SwerveModuleState[] states = toSwerveModuleStates(speeds);
        
        swerve.outputModuleStates(
            states,
            true, 
            false
        );
    }

    public void driveWithHeading(double x, double y, double targetHeading) {
        double[] inputs = convertInputs(x, y);

        if (enableSlewRateLimiters) {
            applyLimiters(inputs);
        }

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x, 
            y, 
            thetaController.calculate(
                swerve.getAngle().getDegrees(), targetHeading
            ), 
            swerve.getAngle()
        );

        speeds = correctDynamics(speeds);

        SwerveModuleState states[] = toSwerveModuleStates(speeds);

        swerve.outputModuleStates(states, true, true);
    }

    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds) {
        return Constants.PHYSICAL_CONSTANTS.KINEMATICS.toSwerveModuleStates(speeds);
    }

    public void resetThetaController() {
        thetaController.reset(swerve.getYaw());
    }

    /**
     * 
     * @param x value from joystick
     * @param y value from joystick
     * @return x value at index 0, y value at index 1
     */
    public double[] convertInputs(double x, double y) {
        double[] newInputs = {
            x * Constants.PHYSICAL_CONSTANTS.MAX_TRANSLATION_METERS,
            y * Constants.PHYSICAL_CONSTANTS.MAX_TRANSLATION_METERS,
        };
 
        return newInputs;
    }

    /**
     * @param x value from joystick
     * @param y value from joystick
     * @param theta value from joystick
     * @return x value at index 0, y value at index 1, theta value at index 2
     */
    public double[] convertInputs(double x, double y, double theta) {
        double[] newInputs = {
            x * Constants.PHYSICAL_CONSTANTS.MAX_TRANSLATION_METERS,
            y * Constants.PHYSICAL_CONSTANTS.MAX_TRANSLATION_METERS,
            theta * Constants.PHYSICAL_CONSTANTS.MAX_ANGULAR_SPEED_RAD
        };
 
        return newInputs;
    }

    public double[] applyLimiters(double[] values) {
        double inputs[] = new double[3];
        
        inputs[0] = xLimiter.calculate(values[0]);
        inputs[1] = yLimiter.calculate(values[1]);
        
        if (inputs.length == 3){ 
            thetaLimiter.calculate(values[2]);
        }

        return inputs;
    }

    public ChassisSpeeds correctDynamics(ChassisSpeeds desiredSpeeds) {
        Pose2d futurePose = 
            new Pose2d(
                desiredSpeeds.vxMetersPerSecond * Constants.LOOP_TIME_S,
                desiredSpeeds.vyMetersPerSecond * Constants.LOOP_TIME_S,
                Rotation2d.fromRadians(desiredSpeeds.omegaRadiansPerSecond * Constants.LOOP_TIME_S)
            );

        Twist2d twistForPose = log(futurePose);
        
        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(
            twistForPose.dx / Constants.LOOP_TIME_S,
            twistForPose.dy / Constants.LOOP_TIME_S,
            twistForPose.dtheta / Constants.LOOP_TIME_S
        );

        return updatedChassisSpeeds;
    } 

    /**
    * Borrowed from 254:
    * https://github.com/Team254/FRC-2022-Public/blob/b5da3c760b78d598b492e1cc51d8331c2ad50f6a/src/main/java/com/team254/lib/geometry/Pose2d.java
    */
    private Twist2d log(final Pose2d transform) {
        double kEps = 1E-9;

        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta =
                -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
        }
        final Translation2d translation_part =
            transform
                .getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }
}