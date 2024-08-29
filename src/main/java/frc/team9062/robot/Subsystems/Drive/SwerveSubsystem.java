package frc.team9062.robot.Subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Util.CriticalSubsystem;
import frc.team9062.robot.Util.SystemState;
import frc.team9062.robot.Util.SystemState.VERBOSITY_LEVEL;
import frc.team9062.robot.Util.lib.LimelightHelpers;

public class SwerveSubsystem extends CriticalSubsystem {
    private static SwerveSubsystem instance;
    private Module frontleft, frontright, rearleft, rearright;
    private AHRS gyro;
    private Pose2d pose = new Pose2d(1.2, 5.55, Rotation2d.fromDegrees(180));
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field2d;
    private Field2d est_field;
    private VERBOSITY_LEVEL verbosity;
    private boolean flipAutonPath = true;

    public SwerveSubsystem() {
        frontleft = new Module(
            "front left", 
            Constants.DEVICE_IDs.FRONT_LEFT_DRIVE_ID, 
            Constants.DEVICE_IDs.FRONT_LEFT_TURN_ID, 
            Constants.DEVICE_IDs.FRONT_LEFT_CANCODER_ID, 
            Constants.PHYSICAL_CONSTANTS.FRONT_LEFT_OFFSET, 
            false, 
            true
        );

        frontright = new Module(
            "front right", 
            Constants.DEVICE_IDs.FRONT_RIGHT_DRIVE_ID, 
            Constants.DEVICE_IDs.FRONT_RIGHT_TURN_ID, 
            Constants.DEVICE_IDs.FRONT_RIGHT_CANCODER_ID, 
            Constants.PHYSICAL_CONSTANTS.FRONT_RIGHT_OFFSET, 
            true, 
            true
        );

        rearleft = new Module(
            "rear left", 
            Constants.DEVICE_IDs.REAR_LEFT_DRIVE_ID, 
            Constants.DEVICE_IDs.REAR_LEFT_TURN_ID, 
            Constants.DEVICE_IDs.REAR_LEFT_CANCODER_ID, 
            Constants.PHYSICAL_CONSTANTS.REAR_LEFT_OFFSET, 
            false, 
            true
        );

        rearright = new Module(
            "rear right", 
            Constants.DEVICE_IDs.REAR_RIGHT_DRIVE_ID, 
            Constants.DEVICE_IDs.REAR_RIGHT_TURN_ID, 
            Constants.DEVICE_IDs.REAR_RIGHT_CANCODER_ID, 
            Constants.PHYSICAL_CONSTANTS.REAR_RIGHT_OFFSET, 
            true, 
            true
        );

        
        gyro = new AHRS(Constants.DEVICE_IDs.GYRO_PORT);
        gyro.setAngleAdjustment(0);

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
                    //odometry.resetPosition(getAngle(), getSwerveModulePositions(), pose);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        ).start();

        reset();

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
            DriverStation.getAlliance().get() == Alliance.Blue ? pose : new Pose2d(Constants.PHYSICAL_CONSTANTS.FIELD_LENGTH_METERS - pose.getX(), pose.getY(), Rotation2d.fromDegrees(pose.getRotation().getDegrees() + 180))
            //VecBuilder.fill(0.01, 0.01, 0.03),
            //VecBuilder.fill(0.02, 0.02, 0.2)
        );

        field2d = new Field2d();
        est_field = new Field2d();

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdom, 
            this::getChassisSpeeds, 
            this::outputModuleStates, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(2.5),
                new PIDConstants(6),
                Constants.PHYSICAL_CONSTANTS.MAX_WHEEL_SPEED_METERS, 
                Constants.PHYSICAL_CONSTANTS.TRACK_RADIUS_METERS,
                new ReplanningConfig(), 
                Constants.LOOP_TIME_S
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this
        );
    }

    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }

        return instance;
    }

    public double getYaw() {
        return gyro.getAngle() * Constants.PHYSICAL_CONSTANTS.GYRO_REVERSED;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getPoseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.PHYSICAL_CONSTANTS.KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
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

    public boolean isAutoPathFlipped() {
        return flipAutonPath;
    }
    
    public void flipAutonPath(boolean flip) {
        flipAutonPath = flip;
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public void reset() {
        //zeroGyro();
        frontleft.reset();
        frontright.reset();
        rearleft.reset();
        rearright.reset();
    }

    public void resetOdom(Pose2d pose) {
        odometry.resetPosition(getAngle(), getSwerveModulePositions(), pose);
    }

    public void resetPoseEstimate(Pose2d pose) {
        poseEstimator.resetPosition(getAngle(), getSwerveModulePositions(), pose);
    }

    public void updatePoseEstimator() {
        poseEstimator.update(getAngle(), getSwerveModulePositions());
    }

    public void updatePoseEstimatorWithVision() {
        poseEstimator.update(getAngle(), getSwerveModulePositions());

        /* 
        if (SystemState.getInstance().getShootingData(() -> getPoseEstimate()).distance > 3) {
            LimelightHelpers.setPipelineIndex("limelight", 1);
        } else {
            LimelightHelpers.setPipelineIndex("limelight", 0);
        }
        */

        if (LimelightHelpers.getTV("limelight")) {
            poseEstimator.addVisionMeasurement(
                LimelightHelpers.getBotPose2d_wpiBlue("limelight"), //DriverStation.getAlliance().get() == Alliance.Blue ? LimelightHelpers.getBotPose2d_wpiBlue("limelight") : LimelightHelpers.getBotPose2d_wpiRed("limelight"), 
                Timer.getFPGATimestamp(),
                VecBuilder.fill(0.01, 0.01, 0.2)
            );
        }
    }

    public void setDriveBrake(boolean brake) {
        frontleft.setDriveBrake(brake);
        frontright.setDriveBrake(brake);
        rearleft.setDriveBrake(brake);
        rearright.setDriveBrake(brake);
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

    public void outputModuleStates(ChassisSpeeds speeds) {
        SwerveModuleState states[] = Constants.PHYSICAL_CONSTANTS.KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.PHYSICAL_CONSTANTS.MAX_WHEEL_SPEED_METERS);

        frontleft.setSwerveModuleState(states[0], true, false);
        frontright.setSwerveModuleState(states[1], true, false);
        rearleft.setSwerveModuleState(states[2], true, false);
        rearright.setSwerveModuleState(states[3], true, false);
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

    public void telemetry() {
        SmartDashboard.putData("Field", field2d);
        SmartDashboard.putData("Estimated Pose", est_field);

        SmartDashboard.putNumber("POSE X", getPose().getX());
        SmartDashboard.putNumber("POSE Y", getPose().getY());
        SmartDashboard.putNumber("POSE THETA", getPose().getRotation().getDegrees());

        if (verbosity == VERBOSITY_LEVEL.LOW) {
            frontleft.telemetry(VERBOSITY_LEVEL.LOW);
            frontright.telemetry(VERBOSITY_LEVEL.LOW);
            rearleft.telemetry(VERBOSITY_LEVEL.LOW);
            rearright.telemetry(VERBOSITY_LEVEL.LOW);
        }
        
        if(verbosity == VERBOSITY_LEVEL.HIGH) {
            frontleft.telemetry(VERBOSITY_LEVEL.HIGH);
            frontright.telemetry(VERBOSITY_LEVEL.HIGH);
            rearleft.telemetry(VERBOSITY_LEVEL.HIGH);
            rearright.telemetry(VERBOSITY_LEVEL.HIGH);
        }
    }

    @Override
    public void periodic() {
        //updatePoseEstimator();
        updatePoseEstimatorWithVision();
        odometry.update(getAngle(), getSwerveModulePositions());
        field2d.setRobotPose(getPose());
        est_field.setRobotPose(getPoseEstimate());

        telemetry();
    }
}