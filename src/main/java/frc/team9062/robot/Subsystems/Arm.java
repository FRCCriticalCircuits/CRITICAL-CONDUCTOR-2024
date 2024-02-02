package frc.team9062.robot.Subsystems;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Constants;

public class Arm extends SubsystemBase {
    private static Arm instance;
    private CANSparkMax arm;
    private SparkPIDController armPID;
    private SparkAbsoluteEncoder absoluteEncoder;
    private RelativeEncoder encoder;
    private ArmFeedforward arbfeedforward;
    private TrapezoidProfile motionProfile;
    private double timestamp, lastPosition;
    private boolean useLastSetpoint = false;
    private ARM_STATE arm_state = ARM_STATE.MANUAL;

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

        encoder.setPositionConversionFactor((1 / Constants.PHYSICAL_CONSTANTS.ARM_GEAR_RATIO) * Math.PI * 2);
        encoder.setVelocityConversionFactor(((1 / Constants.PHYSICAL_CONSTANTS.ARM_GEAR_RATIO) * Math.PI * 2) / 60);

        absoluteEncoder.setPositionConversionFactor(1 / 2);
        absoluteEncoder.setVelocityConversionFactor((1 / 2) / 60);

        encoder.setPosition(Constants.PHYSICAL_CONSTANTS.ARM_HOLD_POSITION); // Todo: Replace with absolute angle when throughbore is installed

        absoluteEncoder.setZeroOffset(0);

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
                Math.PI / 2, 
                Math.PI
            )
        );

        arm.setIdleMode(IdleMode.kBrake);

        arm.burnFlash();
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }

        return instance;
    }

    public void setArmPosition(double position) {
        if (position != lastPosition) {
            timestamp = Timer.getFPGATimestamp();
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

        armPID.setReference(
            position, 
            ControlType.kPosition,
            0,
            arbfeedforward.calculate(
                state.position, 
                state.velocity, 
                position
            ),
            ArbFFUnits.kVoltage
        );

        lastPosition = position;
    }

    public void setPositionSmartMotion(double position) {
        armPID.setReference(
            position,
            ControlType.kPosition,
            0
        );

        lastPosition = position;
    }

    public void setPositionSmartMotion(Rotation2d position) {
        armPID.setReference(
            position.getRadians(),
            ControlType.kSmartMotion,
            0
        );

        lastPosition = position.getRadians();
    }
    
    public void useLastSetpoint(boolean useLastSetpoint) {
        this.useLastSetpoint = useLastSetpoint;
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
        return Math.toRadians(absoluteEncoder.getPosition());
    }

    public double getAbsolutePositionDeg() {
        return absoluteEncoder.getPosition();
    }

    public void setRaw(double speed) {
        arm.set(speed);
    }

    public double getCurrent(int current) {
        return arm.getOutputCurrent();
    }

    public void telemetry() {
        SmartDashboard.putNumber("ARM ANGLE", getPositionDeg());
        SmartDashboard.putNumber("ARM ANGLE RAD", getPositionRad());

        SmartDashboard.putNumber("ARM VEL", getRadPerSec());
    }

    @Override
    public void periodic() {
        if (useLastSetpoint) {
            if (arm_state == ARM_STATE.ALIGNING) {
                setArmPosition(lastPosition);
            }else if(arm_state == ARM_STATE.AIMING){
                setPositionSmartMotion(lastPosition);
            }
        }

        telemetry();
    }

    public enum ARM_STATE {
        MANUAL,
        ALIGNING,
        AIMING,
        SETPOINT
    }
}