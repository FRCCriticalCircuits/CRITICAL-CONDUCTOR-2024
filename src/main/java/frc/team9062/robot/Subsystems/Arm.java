package frc.team9062.robot.Subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.Drive.SwerveSubsystem;
import frc.team9062.robot.Util.CriticalSubsystem;

public class Arm extends CriticalSubsystem {
    private static Arm instance;
    private CANSparkMax arm;
    private SparkPIDController armPID;
    private SparkAbsoluteEncoder absoluteEncoder;
    private RelativeEncoder encoder;
    private ArmFeedforward arbfeedforward;
    private TrapezoidProfile motionProfile;
    private double timestamp, accel, custom_position, position = Constants.PHYSICAL_CONSTANTS.ARM_HOLD_POSITION_HIGH, lastvel;
    private ARM_STATE arm_state = ARM_STATE.MANUAL;
    private State setpoint;

    private static Map<ARM_STATE, Double> arm_map = new HashMap<>();

    static {
        arm_map.put(ARM_STATE.INTAKE, Constants.PHYSICAL_CONSTANTS.ARM_INTAKE_POSITION);
        arm_map.put(ARM_STATE.AMP, Constants.PHYSICAL_CONSTANTS.ARM_AMP_POSITION);
        arm_map.put(ARM_STATE.LOW, Constants.PHYSICAL_CONSTANTS.ARM_HOLD_POSITION_LOw);
        arm_map.put(ARM_STATE.HIGH, Constants.PHYSICAL_CONSTANTS.ARM_HOLD_POSITION_HIGH);
        arm_map.put(ARM_STATE.PREPARE_HOOK, 1.0);
        arm_map.put(ARM_STATE.HOOK, 0.0);
    }

    public Arm() {
        arm = new CANSparkMax(
            Constants.DEVICE_IDs.ARM_ID,
            MotorType.kBrushless
        );

        arm.restoreFactoryDefaults();
        arm.setSmartCurrentLimit(Constants.PHYSICAL_CONSTANTS.ARM_CURRENT_LIMIT);
        arm.enableVoltageCompensation(Constants.PHYSICAL_CONSTANTS.NOMINAL_VOLTAGE);

        // -----ENCODERS-----^
        
        encoder = arm.getEncoder();
        absoluteEncoder = arm.getAbsoluteEncoder(Type.kDutyCycle);

        encoder.setPositionConversionFactor(1.0 / Constants.PHYSICAL_CONSTANTS.ARM_GEAR_RATIO * Math.PI * 2);
        encoder.setVelocityConversionFactor((1.0 / Constants.PHYSICAL_CONSTANTS.ARM_GEAR_RATIO * Math.PI * 2) / 60);

        absoluteEncoder.setPositionConversionFactor((15.0 / 48.0) * 2.0 * Math.PI);
        absoluteEncoder.setVelocityConversionFactor(((15.0 / 48.0) * 2.0 * Math.PI) / 60);

        absoluteEncoder.setInverted(true);

        absoluteEncoder.setZeroOffset(Constants.PHYSICAL_CONSTANTS.ARM_OFFSET);

        // ------------------ 

        // ------PID------

        armPID = arm.getPIDController();
        
        armPID.setP(Constants.TUNED_CONSTANTS.ARM_PIDF0_P, 0);
        armPID.setI(Constants.TUNED_CONSTANTS.ARM_PIDF0_I, 0);
        armPID.setD(Constants.TUNED_CONSTANTS.ARM_PIDF0_D, 0);
        armPID.setFF(Constants.TUNED_CONSTANTS.ARM_PIDF0_F, 0);

        armPID.setP(Constants.TUNED_CONSTANTS.ARM_PIDF1_P, 1);
        armPID.setI(Constants.TUNED_CONSTANTS.ARM_PIDF1_I, 1);
        armPID.setD(Constants.TUNED_CONSTANTS.ARM_PIDF1_D, 1);
        armPID.setFF(Constants.TUNED_CONSTANTS.ARM_PIDF1_F, 1);

        armPID.setFeedbackDevice(encoder);

        //armPID.setOutputRange(-1, 1);

        armPID.setSmartMotionMaxVelocity(Constants.TUNED_CONSTANTS.ARM_MAX_RAD, 0);
        armPID.setSmartMotionMaxAccel(Constants.TUNED_CONSTANTS.ARM_MAX_RAD_2, 0);

        arbfeedforward = new ArmFeedforward(
            Constants.TUNED_CONSTANTS.ARM_FF0_KS, 
            Constants.TUNED_CONSTANTS.ARM_FF0_KG, 
            Constants.TUNED_CONSTANTS.ARM_FF0_KV,
            Constants.TUNED_CONSTANTS.ARM_FF0_KA
        );

        // ---------------

        motionProfile = new TrapezoidProfile(
            new Constraints(
                Math.PI, 
                Math.PI * 2
            )
        );

        arm.setInverted(true);
        arm.setIdleMode(IdleMode.kBrake);

        arm.burnFlash();

        new Thread(
            () -> {
                try {
                    Timer.delay(2);
                    encoder.setPosition(getAbsolutePositionRad());
                    setpoint = new State(getPositionRad(), getRadPerSec());
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        ).start();
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }

        return instance;
    }

    public void setArmPosition(double position) {
        this.position = position;
        
        /* 
        if (this.position != position) {
            timestamp = Timer.getFPGATimestamp();
            this.position = position;
        }

        State state = motionProfile.calculate(
            Timer.getFPGATimestamp() - timestamp, 
            new State(
                getPositionRad(), 
                getRadPerSec()
            ),
            new State(
                position, 
                0
            )
        );
        */

        setpoint = motionProfile.calculate(
            0.02, 
            setpoint, 
            new State(
                position, 
                0
            )
        );

        // To stop arm from slamming ground or moving past upright position
        /* 
        if (Math.abs(state.position - getSetpoint()) > Math.abs(getPositionRad() - getSetpoint())) {
            state.velocity = 0;
        }
        */

        armPID.setReference(
            setpoint.position, 
            ControlType.kPosition,
            0,
            arbfeedforward.calculate(
                setpoint.position,
                setpoint.velocity
            ),
            ArbFFUnits.kVoltage
        );
    }

    public void setArmPosition(double position, boolean useShooterAngle) {
        this.position = position;

        State state = motionProfile.calculate(
            Constants.LOOP_TIME_S, 
            new State(
                getPositionRad(), 
                getRadPerSec()), 
            new State(
                useShooterAngle ? position + Math.toRadians(35) : position,
                0
            )
        );

        armPID.setReference(
            state.position, 
            ControlType.kPosition,
            0,
            arbfeedforward.calculate(
                state.position,
                state.velocity
            ),
            ArbFFUnits.kVoltage
        );
    }

    public void setPositionSmartMotion(double position) {
        armPID.setReference(
            position,
            ControlType.kPosition,
            0
        );
    }

    public void setPositionSmartMotion(Rotation2d position) {
        armPID.setReference(
            position.getRadians(),
            ControlType.kSmartMotion,
            0
        );
    }

    public void setArmVelocity(double velocity) {
        armPID.setReference(
            velocity,
            ControlType.kVelocity,
            1,
            arbfeedforward.calculate(
                getPositionRad(), 
                velocity
            ),
            ArbFFUnits.kVoltage    
        );
    }

    public void aimForTarget(Supplier<Pose2d> robotPose) {
        Translation2d robotToTarget = new Translation2d(
            DriverStation.getAlliance().get() == Alliance.Blue ? robotPose.get().getX() - Constants.PHYSICAL_CONSTANTS.CENTRE_SPEAKER_OPENING.getX() : Constants.PHYSICAL_CONSTANTS.FIELD_LENGTH_METERS - Constants.PHYSICAL_CONSTANTS.CENTRE_SPEAKER_OPENING.getX() - robotPose.get().getX(), 
            Constants.PHYSICAL_CONSTANTS.CENTRE_SPEAKER_OPENING.getZ() - Constants.PHYSICAL_CONSTANTS.ARM_HEIGHT_METERS
        );

        double targetToShooterTheta = Math.asin((Constants.PHYSICAL_CONSTANTS.ARM_LENGTH_METERS * Math.sin(65)) / robotToTarget.getNorm());

        double angle = Math.toRadians(65) - Math.toRadians(180 - targetToShooterTheta - (180 - robotToTarget.getAngle().getDegrees()));

        setArmPosition(angle);
    }
    
    public void setArmState(ARM_STATE state) {
        if (arm_state != state) {
            arm_state = state;
        }
    }

    public void setCustomSetpoint(double custom_position) {
        this.custom_position = custom_position;
        arm_state = ARM_STATE.CUSTOM;
    }

    public double getShooterAngleRad() {
        return getPositionRad() + Math.toRadians(35);
    }

    public double getShooterAngleDeg() {
        return getPositionDeg() + 35;
    }

    public double getPositionRad() {
        return encoder.getPosition();
    }

    public double getRadPerSec() {
        return encoder.getVelocity();
    }

    public double getPositionDeg() {
        return Math.toDegrees(encoder.getPosition());
    }

    public double getDegPerSec() {
        return Math.toDegrees(encoder.getVelocity());
    }

    public double getAbsolutePositionRad() {
        return absoluteEncoder.getPosition();
    }

    public double getAbsolutePositionDeg() {
        return Math.toDegrees(absoluteEncoder.getPosition());
    }

    public double getArmRadPerSec2() {
        return accel;
    }

    public double getSetpoint() {
        return position;
    }

    public boolean atSetpoint() {
        if (getArmRadPerSec2() < Constants.PHYSICAL_CONSTANTS.ARM_ACCEL_SETPOINT_THRESHOLD 
                && Math.abs(getSetpoint() - getPositionRad()) < Constants.PHYSICAL_CONSTANTS.ARM_SETPOINT_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    public ARM_STATE getArmState() {
        return arm_state;
    }

    public double getCurrent(int current) {
        return arm.getOutputCurrent();
    }

    public void calculate_accel() {
        accel = (getRadPerSec() - lastvel) / Constants.LOOP_TIME_S;
    }

    public void setRaw(double speed) {
        arm.set(speed);
    }

    public void telemetry() {
        SmartDashboard.putNumber("ARM ANGLE", getPositionDeg());
        SmartDashboard.putNumber("ARM ANGLE RAD", getPositionRad());
        SmartDashboard.putNumber("ARM ABSOLUTE ANGLE", getAbsolutePositionRad());

        SmartDashboard.putNumber("ARM ACCEL", getArmRadPerSec2());

        SmartDashboard.putBoolean("AT SETPOINT[ARM]", atSetpoint());
    }

    @Override
    public void periodic() {
        /* 
        switch (arm_state) {
            case LOW:
                setArmPosition(arm_map.get(arm_state));
                break;
            case HIGH:
                setArmPosition(arm_map.get(arm_state));
                break;
            case INTAKE:
                setArmPosition(arm_map.get(arm_state));
                break;
            case AMP:
                setArmPosition(arm_map.get(arm_state));
                break;
            case CUSTOM:
                setArmPosition(custom_position);
                break;
            case AIM:
                aimForTarget(SwerveSubsystem.getInstance()::getPose);
                break;
            default:
                break;
        }
        */

        calculate_accel();
        lastvel = getRadPerSec();

        telemetry();
    }

    @Override
    public void handleStates() {
        switch (arm_state) {
            case LOW -> {
                setArmPosition(arm_map.get(arm_state));
            }
            case HIGH -> {
                setArmPosition(arm_map.get(arm_state));
            }
            case INTAKE -> {
                setArmPosition(arm_map.get(arm_state));
            }
            case AMP -> {
                setArmPosition(arm_map.get(arm_state));
            }
            case CUSTOM -> {
                setArmPosition(custom_position);
            }
            case AIM -> {
                aimForTarget(SwerveSubsystem.getInstance()::getPose);
            }
            default -> {}
        }
    }

    public enum ARM_STATE {
        MANUAL,
        LOW,
        HIGH,
        INTAKE,
        AMP,
        PREPARE_HOOK,
        HOOK,
        AIM,
        CUSTOM
    }
}