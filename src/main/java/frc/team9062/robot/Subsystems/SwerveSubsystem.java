package frc.team9062.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Util.RobotState.VERBOSITY_LEVEL;

public class SwerveSubsystem extends SubsystemBase{
    private static SwerveSubsystem instance;
    private Module frontleft, frontright, rearleft, rearright;
    private AHRS gyro;
    private Pose2d pose = new Pose2d();
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field2d;
    

    public SwerveSubsystem() {
        frontleft = new Module(
            "front left", 
            Constants.DEVICE_IDs.FRONT_LEFT_DRIVE_ID, 
            Constants.DEVICE_IDs.FRONT_LEFT_TURN_ID, 
            Constants.DEVICE_IDs.FRONT_LEFT_CANCODER_ID, 
            Constants.PHYSICAL_CONSTANTS.FRONT_LEFT_OFFSET, 
            false, 
            true, 
            VERBOSITY_LEVEL.COMP
        );

        frontright = new Module(
            "front right", 
            Constants.DEVICE_IDs.FRONT_RIGHT_DRIVE_ID, 
            Constants.DEVICE_IDs.FRONT_RIGHT_TURN_ID, 
            Constants.DEVICE_IDs.FRONT_RIGHT_CANCODER_ID, 
            Constants.PHYSICAL_CONSTANTS.FRONT_RIGHT_OFFSET, 
            false, 
            true, 
            VERBOSITY_LEVEL.COMP
        );

        rearleft = new Module(
            "rear left", 
            Constants.DEVICE_IDs.REAR_LEFT_DRIVE_ID, 
            Constants.DEVICE_IDs.REAR_LEFT_TURN_ID, 
            Constants.DEVICE_IDs.REAR_LEFT_CANCODER_ID, 
            Constants.PHYSICAL_CONSTANTS.REAR_LEFT_OFFSET, 
            false, 
            true, 
            VERBOSITY_LEVEL.COMP
        );

        rearright = new Module(
            "rear right", 
            Constants.DEVICE_IDs.REAR_RIGHT_DRIVE_ID, 
            Constants.DEVICE_IDs.REAR_RIGHT_TURN_ID, 
            Constants.DEVICE_IDs.REAR_RIGHT_CANCODER_ID, 
            Constants.PHYSICAL_CONSTANTS.REAR_RIGHT_OFFSET, 
            false, 
            true, 
            VERBOSITY_LEVEL.COMP
        );

        
        gyro = new AHRS(Constants.DEVICE_IDs.GYRO_PORT);

        gyro.calibrate();

        new Thread(
            () -> {
                try {
                    Thread.sleep(10);
                    frontleft.reset();
                    Thread.sleep(10);
                    frontright.reset();
                    Thread.sleep(10);
                    rearleft.reset();
                    Thread.sleep(10);
                    rearright.reset();
                    Thread.sleep(1000);
                    gyro.reset();
                    odometry.resetPosition(getAngle(), getSwerveModulePositions(), pose);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        ).start();

        odometry = new SwerveDriveOdometry(
            Constants.PHYSICAL_CONSTANTS.KINEMATICS,
            getAngle(),
            getSwerveModulePositions(), 
            pose
        );
        
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.PHYSICAL_CONSTANTS.KINEMATICS, 
            getAngle(), 
            getSwerveModulePositions(), 
            pose
        );

        field2d = new Field2d();
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }

        return instance;
    }

    public double getYaw() {
        return gyro.getYaw();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void updatePoseEstimator() {
        poseEstimator.update(getAngle(), getSwerveModulePositions());
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = {
            frontleft.getSwerveModulePosition(),
            frontright.getSwerveModulePosition(),
            rearleft.getSwerveModulePosition(),
            rearright.getSwerveModulePosition()
        }; 

        return positions;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = {
            frontleft.getSwerveModuleState(),
            frontright.getSwerveModuleState(),
            rearleft.getSwerveModuleState(),
            rearright.getSwerveModuleState()
        };
        
        return states;
    }

    public void stop() {
        SwerveModuleState[] states = {
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0))
        };

        outputModuleStates(
            states, 
            true, 
            true
        );
    }

    public void outputModuleStates(SwerveModuleState[] states, boolean velocityControl, boolean antiJitter) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, 
            Constants.PHYSICAL_CONSTANTS.MAX_WHEEL_SPEED_METERS
        );
        
        frontleft.setSwerveModuleState(states[0], velocityControl, antiJitter);
        frontright.setSwerveModuleState(states[1], velocityControl, antiJitter);
        rearleft.setSwerveModuleState(states[2], velocityControl, antiJitter);
        rearright.setSwerveModuleState(states[3], velocityControl, antiJitter);
    }

    @Override
    public void periodic() {
        odometry.update(getAngle(), getSwerveModulePositions());

        field2d.setRobotPose(getPose());
        SmartDashboard.putData(field2d);

        SmartDashboard.putNumber("ABS FL", frontleft.getAbsoluteAngle());
        SmartDashboard.putNumber("ABS FR", frontright.getAbsoluteAngle());
        SmartDashboard.putNumber("ABS RL", rearleft.getAbsoluteAngle());
        SmartDashboard.putNumber("ABS RR", rearright.getAbsoluteAngle());
    }
}