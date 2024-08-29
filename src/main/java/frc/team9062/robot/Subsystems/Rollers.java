package frc.team9062.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.Drive.SwerveSubsystem;
import frc.team9062.robot.Util.SystemState;

public class Rollers extends SubsystemBase {
    private static Rollers instance;
    private CANSparkMax shooter, shooter_follower;
    private TalonFX intake;
    private RelativeEncoder intake_encoder, shooter_encoder;
    private SparkPIDController intakePID, shooterPID;
    private SimpleMotorFeedforward shooter_arbFeedforward, intake_arbFeedforward;
    private boolean smartIntake = true;
    private double shooter_accel, intake_accel, lastvel_shooter = 0, lastvel_intake = 0, shooter_setpoint = 0;
    private INTAKE_STATE intake_state = INTAKE_STATE.IDLE;
    private MedianFilter currentFilter = new MedianFilter(150);
    private DigitalInput sensor_left = new DigitalInput(0), sensor_right = new DigitalInput(1);
    private Debouncer left_debouncer = new Debouncer(Constants.INTAKE_DEBOUNCE_TIME, DebounceType.kFalling);
    private Debouncer right_debouncer = new Debouncer(Constants.INTAKE_DEBOUNCE_TIME, DebounceType.kFalling);

    public Rollers() {
        intake = new TalonFX(Constants.DEVICE_IDs.ROLLER_ID);
        shooter = new CANSparkMax(Constants.DEVICE_IDs.SHOOTER_ID, MotorType.kBrushless);
        shooter_follower = new CANSparkMax(Constants.DEVICE_IDs.SHOOTER_FOLLOWER_ID, MotorType.kBrushless);

        //intake.restoreFactoryDefaults();
        shooter.restoreFactoryDefaults();
        shooter_follower.restoreFactoryDefaults();

        intake.setInverted(true);
        shooter.setInverted(true);

        // -----MOTORS-----

        // --- INTAKE CONFIG ---

        TalonFXConfiguration intake_config = new TalonFXConfiguration();

        intake_config.CurrentLimits.withSupplyCurrentLimit(Constants.PHYSICAL_CONSTANTS.INTAKE_CURRENT_LIMIT);
        intake_config.CurrentLimits.withSupplyCurrentLimitEnable(true);

        intake_config.Feedback.withSensorToMechanismRatio(Constants.PHYSICAL_CONSTANTS.INTAKE_GEAR_RATIO * (1 / (Constants.PHYSICAL_CONSTANTS.INTAKE_WHEEL_DIAMETER_FEET * Math.PI)));

        intake_config.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        intake_config.Slot0.withKP(4);
        intake_config.Slot0.withKI(0);
        intake_config.Slot0.withKD(0.4);

        // --------------------

        intake.getConfigurator().apply(intake_config);
        shooter.setSmartCurrentLimit(Constants.PHYSICAL_CONSTANTS.SHOOTER_CURRENT_LIMIT);
        shooter_follower.setSmartCurrentLimit(Constants.PHYSICAL_CONSTANTS.SHOOTER_CURRENT_LIMIT);

        //intake.enableVoltageCompensation(Constants.PHYSICAL_CONSTANTS.NOMINAL_VOLTAGE);
        shooter.enableVoltageCompensation(Constants.PHYSICAL_CONSTANTS.NOMINAL_VOLTAGE);
        shooter_follower.enableVoltageCompensation(Constants.PHYSICAL_CONSTANTS.NOMINAL_VOLTAGE);

        //intake.setClosedLoopRampRate(0.2);
        shooter.setClosedLoopRampRate(0.1);
        shooter_follower.setClosedLoopRampRate(0.1);

        shooter_follower.follow(shooter, true);

        // ----------------

        // -----ENCODERS-----

        //intake_encoder = intake.getEncoder();
        shooter_encoder = shooter.getEncoder();

        //intake_encoder.setPositionConversionFactor(1 / Constants.PHYSICAL_CONSTANTS.INTAKE_GEAR_RATIO * Constants.PHYSICAL_CONSTANTS.INTAKE_WHEEL_DIAMETER_FEET * Math.PI);
        //intake_encoder.setVelocityConversionFactor((1 / Constants.PHYSICAL_CONSTANTS.INTAKE_GEAR_RATIO * Constants.PHYSICAL_CONSTANTS.INTAKE_WHEEL_DIAMETER_FEET * Math.PI) / 60);
        
        shooter_encoder.setPositionConversionFactor(Constants.PHYSICAL_CONSTANTS.SHOOTER_GEAR_RATIO * Constants.PHYSICAL_CONSTANTS.SHOOTER_WHEEL_DIAMETER_FEET * Math.PI);
        shooter_encoder.setVelocityConversionFactor((Constants.PHYSICAL_CONSTANTS.SHOOTER_GEAR_RATIO * Constants.PHYSICAL_CONSTANTS.SHOOTER_WHEEL_DIAMETER_FEET * Math.PI) / 60);

        //intake_encoder.setPosition(0);

        // ------------------

        // -----PID-----

        //intakePID = intake.getPIDController();
        shooterPID = shooter.getPIDController();

        //intakePID.setP(Constants.TUNED_CONSTANTS.INTAKE_PIDF0_P, 0);
        //intakePID.setI(Constants.TUNED_CONSTANTS.INTAKE_PIDF0_I, 0);
        //intakePID.setD(Constants.TUNED_CONSTANTS.INTAKE_PIDF0_D, 0);

        shooterPID.setP(Constants.TUNED_CONSTANTS.SHOOTER_PIDF0_P, 0);
        shooterPID.setI(Constants.TUNED_CONSTANTS.SHOOTER_PIDF0_I, 0);
        shooterPID.setD(Constants.TUNED_CONSTANTS.SHOOTER_PIDF0_D, 0);
        shooterPID.setFF(Constants.TUNED_CONSTANTS.SHOOTER_PIDF0_F, 0);

        shooterPID.setOutputRange(-1, 1);

        shooter_arbFeedforward = new SimpleMotorFeedforward(
            Constants.TUNED_CONSTANTS.SHOOTER_FF0_KS, 
            Constants.TUNED_CONSTANTS.SHOOTER_FF0_KV,
            Constants.TUNED_CONSTANTS.SHOOTER_FF0_KA
        );

        intake_arbFeedforward = new SimpleMotorFeedforward(
            Constants.TUNED_CONSTANTS.INTAKE_FF0_KS, 
            Constants.TUNED_CONSTANTS.INTAKE_FF0_KV, 
            Constants.TUNED_CONSTANTS.INTAKE_FF0_KA
        );
        
        shooterPID.setFeedbackDevice(shooter_encoder);
        //intakePID.setFeedbackDevice(intake_encoder);

        // -------------

        //intake.burnFlash();
        shooter.burnFlash();
        shooter_follower.burnFlash();

        //intake.setIdleMode(IdleMode.kBrake);
        shooter.setIdleMode(IdleMode.kCoast);
        shooter_follower.setIdleMode(IdleMode.kCoast);   
    }

    public static Rollers getInstance() {
        if (instance == null) {
            instance = new Rollers();
        }

        return instance;
    }

    public void setShooterVelocity(double velocity) {
        shooter_setpoint = velocity;

        shooterPID.setReference(
            velocity, 
            ControlType.kVelocity,
            0,
            shooter_arbFeedforward.calculate(velocity),
            ArbFFUnits.kVoltage
        );
    }

    public void setIntakeVelocity(double velocity) {
        /*
        intakePID.setReference(
            velocity, 
            ControlType.kVelocity,
            0,
            intake_arbFeedforward.calculate(velocity),
            ArbFFUnits.kVoltage
        );
        */

        intake.setControl(new VelocityVoltage(velocity));
    }

    public void setIntakeState(INTAKE_STATE state) {
        intake_state = state;
    }

    public void setIntakeDirection(boolean direction) {
        intake.setInverted(direction);
    }

    public void enableSmartIntake(boolean smartIntake) {
        this.smartIntake = smartIntake;
    }

    public void intake() {
        if (smartIntake) {
            if (!isWithGamePiece()) {
                setIntakeVelocity(8
                );
            } else {
                setIntakeRaw(0);
            }
        } else {
            setIntakeVelocity(8);
        }
    }

    public void feed() {
        /* 
        if (smartIntake) {
            if (isWithGamePiece()) {
                setIntakeVelocity(2.5);
            } else {
                setIntakeVelocity(0);
            }
        } else {
            setIntakeVelocity(2.5);
        }
        */

        setIntakeVelocity(8);
    }

    public void outake() {
        setIntakeVelocity(-5);
    }
    
    public void setRelativeVelocity() {
        

        setShooterVelocity(intake_accel);
    }

    public void stop() {
        intake.set(0);
        shooter.setVoltage(0);
    }

    public double getShooterVelocity() {
        return shooter_encoder.getVelocity();
    }

    public double getShooterFeetPerMin() {
        return shooter_encoder.getVelocity() * 60;
    }

    public double getIntakeVelocity() {
        return intake.getVelocity().getValue();
    }

    public double getShooterCurrent() {
        return shooter.getOutputCurrent();
    }

    public double getIntakeCurrent() {
        return intake.getStatorCurrent().getValue();
    }

    public double getIntakeFilteredCurrent() {
        return currentFilter.calculate(getIntakeCurrent());
    }

    public double getShooterAccel() {
        return shooter_accel;
    }

    public double getIntakeAccel() {
        return intake_accel;
    }

    public boolean isShooterSpunUp() {
        return Math.abs(shooter_setpoint - getShooterVelocity()) < Constants.PHYSICAL_CONSTANTS.SHOOTER_SETPOINT_THRESHOLD && getShooterAccel() < Constants.PHYSICAL_CONSTANTS.SHOOTER_ACCEL_SETPOINT_THRESHOLD;
    }

    public boolean isWithGamePiece() {
        return left_debouncer.calculate(!sensor_left.get()) || right_debouncer.calculate(!sensor_right.get());
    }

    public void calculate_accel() {
        shooter_accel = (getShooterVelocity() - lastvel_shooter) / Constants.LOOP_TIME_S;
        intake_accel = (getIntakeVelocity() - lastvel_intake) / Constants.LOOP_TIME_MS;

        lastvel_shooter = getShooterVelocity();
        lastvel_intake = getIntakeVelocity();
    }

    public void resetIntakeFilter() {
        currentFilter.reset();
    }

    public void setIntakeRaw(double speed) {
        intake.set(speed);
    }

    public void setShooterRaw(double speed) {
        shooter.set(speed);
    }

    public void telemetry() {
        SmartDashboard.putNumber("SHOOTER VEL[FT/S]", getShooterVelocity());
        SmartDashboard.putNumber("INTAKE VEL[FT/S]", getIntakeVelocity());
        SmartDashboard.putNumber("SHOOTER ACCEL", getShooterAccel());
        SmartDashboard.putNumber("INTAKE ACCEL", getIntakeAccel());

        SmartDashboard.putNumber("INTAKE CURRENT", getIntakeCurrent());
        SmartDashboard.putNumber("INTAKE CURRENT[FILTERED]", getIntakeFilteredCurrent());

        SmartDashboard.putBoolean("GAMEPIECE", isWithGamePiece());

        SmartDashboard.putBoolean("SPUN UP", isShooterSpunUp());
    }

    @Override
    public void periodic() {
        switch (intake_state) {
            case IDLE:
                setIntakeRaw(0);
                break;
            case INTAKING:
                intake();
                break;
            case OUTAKE:
                outake();
                break;
            case FEEDING:
                feed();
                break;
            default:
                break;
        }

        //setShooterVelocity(shooter_setpoint);

        telemetry();
        
        calculate_accel();
    }

    public enum INTAKE_STATE {
        IDLE,
        INTAKING,
        OUTAKE,
        FEEDING
    }
}