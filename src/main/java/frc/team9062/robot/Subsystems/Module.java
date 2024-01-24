package frc.team9062.robot.Subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team9062.robot.Constants;

public class Module {
    String moduleName;
    int driveId, turnId, cancoderId;
    double offset;
    boolean driveReversed, encoderReversed;
    CANSparkMax drive, turn;
    SparkPIDController drivePID, turnPID;
    RelativeEncoder driveEncoder, turnEncoder;
    CANcoder cancoder;
    SimpleMotorFeedforward arbfeedforward;
    SwerveModuleState lastState;
    ShuffleboardTab moduleTab;

    public Module(String moduleName, int driveId, int turnId, int cancoderId, double offset, boolean driveReversed, boolean encoderReversed) {
        this.moduleName = moduleName;
        this.driveId = driveId;
        this.turnId = turnId;
        this.cancoderId = cancoderId;
        this.offset = offset;
        this.driveReversed = driveReversed;
        this.encoderReversed = encoderReversed;

        drive = new CANSparkMax(driveId, MotorType.kBrushless);
        turn = new CANSparkMax(turnId, MotorType.kBrushless);
        cancoder = new CANcoder(cancoderId, "");

        cancoder.getConfigurator().apply(
            new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withSensorDirection(
                    encoderReversed ? 
                        SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive
                ).withMagnetOffset(offset)
        );

        arbfeedforward = new SimpleMotorFeedforward(
            Constants.TUNED_CONSTANTS.DRIVE_FEED_FORWARD_KS, 
            Constants.TUNED_CONSTANTS.DRIVE_FEED_FORWARD_KV, 
            Constants.TUNED_CONSTANTS.DRIVE_FEED_FORWARD_KA
        );

        // ------CONFIGURE MOTORS------

        drive.restoreFactoryDefaults();
        turn.restoreFactoryDefaults();

        drive.enableVoltageCompensation(Constants.PHYSICAL_CONSTANTS.NOMINAL_VOLTAGE);
        turn.enableVoltageCompensation(Constants.PHYSICAL_CONSTANTS.NOMINAL_VOLTAGE);

        drive.setSmartCurrentLimit(Constants.PHYSICAL_CONSTANTS.DRIVE_CURRENT_LIMIT);
        turn.setSmartCurrentLimit(Constants.PHYSICAL_CONSTANTS.TURN_CURRENT_LIMIT);

        drive.setIdleMode(IdleMode.kBrake);
        turn.setIdleMode(IdleMode.kBrake);

        drive.setInverted(driveReversed);
        turn.setInverted(true);
        
        // ----------------------------

        // ------CONFIGURE ENCODERS------

        driveEncoder = drive.getEncoder();

        driveEncoder.setPositionConversionFactor(Units.inchesToMeters(1.0 / Constants.PHYSICAL_CONSTANTS.DRIVE_GEAR_RATIO * Math.PI * 4.0));
        driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(1.0 / Constants.PHYSICAL_CONSTANTS.DRIVE_GEAR_RATIO * Math.PI * 4.0) / 60.0);

        turnEncoder = turn.getEncoder();

        turnEncoder.setPositionConversionFactor((1.0 / Constants.PHYSICAL_CONSTANTS.TURN_GEAR_RATIO) * Math.PI * 2.0);
        turnEncoder.setVelocityConversionFactor(((1.0 / Constants.PHYSICAL_CONSTANTS.TURN_GEAR_RATIO) * Math.PI * 2.0) / 60.0);

        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteAngleRad());

        // ----------------------------

        // ------CONFIGURE PID------

        drivePID = drive.getPIDController();
        turnPID = turn.getPIDController();

        drivePID.setP(Constants.TUNED_CONSTANTS.DRIVE_PIDF0_P, 0);
        drivePID.setI(Constants.TUNED_CONSTANTS.DRIVE_PIDF0_I, 0);
        drivePID.setD(Constants.TUNED_CONSTANTS.DRIVE_PIDF0_D, 0);
        drivePID.setFF(Constants.TUNED_CONSTANTS.DRIVE_PIDF0_F, 0);

        turnPID.setP(Constants.TUNED_CONSTANTS.TURN_PIDF0_P, 0);
        turnPID.setI(Constants.TUNED_CONSTANTS.TURN_PIDF0_I, 0);
        turnPID.setD(Constants.TUNED_CONSTANTS.TURN_PIDF0_D, 0);
        turnPID.setFF(Constants.TUNED_CONSTANTS.TURN_PIDF0_F, 0);

        // -------------------------

        lastState = new SwerveModuleState(
            0, 
            Rotation2d.fromRadians(getAngleRad())
        );

        drive.burnFlash();
        turn.burnFlash();

        moduleTab = Shuffleboard.getTab("Module");
    }

    public void setDriveBrake(boolean brake) {
        if (brake) {
            drive.setIdleMode(IdleMode.kBrake);
        }else {
            drive.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setTurnBrake(boolean brake) {
        if (brake) {
            turn.setIdleMode(IdleMode.kBrake);
        }else {
            turn.setIdleMode(IdleMode.kCoast);
        }
    }

    public double distToTarget(Rotation2d targetAngle) {
        double diff =  targetAngle.getRadians() - getAngle().getRadians();
    
        return diff;
    }

    public double getRelativeTargetAngle(Rotation2d targetAngle) {
        double diff = distToTarget(targetAngle);
        double diff_flipped = distToTarget(Rotation2d.fromRadians(targetAngle.getRadians() + Math.PI));

        diff = Math.min(diff, diff_flipped);

        return getAngleRad() + diff;
    }

    public void setSwerveModuleState(SwerveModuleState state, boolean isOpenLoop, boolean avoidJitter) {
        SwerveModuleState desiredState = SwerveModuleState.optimize(state, getAngle());

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double cosScalar = Math.cos(distToTarget(desiredState.angle));
        cosScalar = cosScalar < 0.0 ? 0.0 : cosScalar;

        double velocity = desiredState.speedMetersPerSecond * cosScalar;

        if (isOpenLoop) {
            drivePID.setReference(
                velocity, 
                ControlType.kVelocity,
                0,
                arbfeedforward.calculate(desiredState.speedMetersPerSecond),
                ArbFFUnits.kVoltage
            );
        }else {
            drivePID.setReference(
                desiredState.speedMetersPerSecond / Constants.PHYSICAL_CONSTANTS.MAX_WHEEL_SPEED_METERS, 
                ControlType.kDutyCycle
            );
        }

        //checkTurnEncoder();

        if (avoidJitter && desiredState.speedMetersPerSecond < Constants.PHYSICAL_CONSTANTS.MAX_WHEEL_SPEED_METERS * 0.01) {
            desiredState.angle = lastState.angle;
        }

        turnPID.setReference(
            getRelativeTargetAngle(desiredState.angle),
            ControlType.kPosition
        );

        lastState = desiredState;
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            getDistance(), 
            getAngle()            
        );
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(
            getVelocity(), 
            getAngle()
        );
    }

    public double getDistance() {
        return driveEncoder.getPosition();
    }

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(getAngleRad());
    }

    public double getAngleDeg() {
        return Math.toDegrees(turnEncoder.getPosition());
    }

    public double getAngleRad() {
        return turnEncoder.getPosition();
    }

    public double getAbsoluteAngle() {
        return cancoder.getAbsolutePosition().getValue();
    }

    public double getAbsoluteAngleDeg() {
        return Math.toDegrees(getAbsoluteAngleRad());
    }

    public double getAbsoluteAngleRad() {
        return cancoder.getAbsolutePosition().getValue() * Math.PI * 2;
    }

    public double getAngleDegPerSec() {
        return Math.toDegrees(turnEncoder.getVelocity());
    }

    public double getAngleRadPerSec() {
        return turnEncoder.getVelocity();
    }

    public void reset() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteAngleRad());
    }

    public void checkTurnEncoder() {
        if(Math.abs(getAngleRad()) > 4*Math.PI) {
            turnEncoder.setPosition(getAbsoluteAngleRad());
        }
    }

    public void setRawDrive(double speed) {
        drive.set(speed);
    }
    
    public void setRawTurn(double speed) {
        turn.set(speed);
    }

    public String getModuleName() {
        return moduleName;
    }

    public int getModuleNumber() {
        return cancoderId/3;
    }

    public void telemetry() {
        
    }
}