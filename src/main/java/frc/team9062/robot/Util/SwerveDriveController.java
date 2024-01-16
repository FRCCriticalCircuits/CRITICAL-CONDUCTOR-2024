package frc.team9062.robot.Util;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    }

    public void drive(double x, double y, double theta, boolean isFieldRelative) {
        double[] inputs = convertInputs(x, y, theta);

        if (enableSlewRateLimiters) {
            applyLimiters(inputs);
        }

        ChassisSpeeds speeds;

        if (isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                inputs[1],
                inputs[0],
                inputs[2], 
                swerve.getAngle()
            );
        } else {
            speeds = new ChassisSpeeds(inputs[1], inputs[0], inputs[2]);
        }

        SwerveModuleState[] states = toSwerveModuleStates(speeds);
        
        swerve.outputModuleStates(
            states, 
            true, 
            true
        );
    }

    public void driveWithHeading(double x, double y, double targetHeading) {}

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
        inputs[2] = thetaLimiter.calculate(values[2]);

        return inputs;
    }
}