package frc.team9062.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class Constants {
    public class DEVICE_IDs {

        // -----------------------------------------------
        // DRIVE ID'Ss
        // -----------------------------------------------

        public static final int FRONT_LEFT_DRIVE_ID = 1;
        public static final int FRONT_LEFT_TURN_ID = 2;
        public static final int FRONT_LEFT_CANCODER_ID = 3;
    
        public static final int FRONT_RIGHT_DRIVE_ID = 4;
        public static final int FRONT_RIGHT_TURN_ID = 5;
        public static final int FRONT_RIGHT_CANCODER_ID = 6;
    
        public static final int REAR_LEFT_DRIVE_ID = 7;
        public static final int REAR_LEFT_TURN_ID = 8;
        public static final int REAR_LEFT_CANCODER_ID = 9;
    
        public static final int REAR_RIGHT_DRIVE_ID = 10;
        public static final int REAR_RIGHT_TURN_ID = 11;
        public static final int REAR_RIGHT_CANCODER_ID = 12;

        // ------------------------------------------------

        // ------------------------------------------------
        // SUPERSTRUCTURE
        // ------------------------------------------------
        
        public static final int ARM_ID = 15;

        public static final int SHOOTER_ID = 20;
        public static final int SHOOTER_FOLLOWER_ID = 21;

        public static final int ROLLER_ID = 17;

        // -------------------------------------------------


        public static Port GYRO_PORT = Port.kUSB;

        public static final int GAMEPAD_DRIVER = 0;
        public static final int GAMEPAD_OPERATOR = 1;

    }

    public class TUNED_CONSTANTS {

        // -------------------------
        // DRIVEBASE
        // -------------------------

        public static final double DRIVE_PIDF0_P = 0.00025596; //4.497E-06;
        public static final double DRIVE_PIDF0_I = 0; 
        public static final double DRIVE_PIDF0_D = 0;
        public static final double DRIVE_PIDF0_F = 0;

        public static final double DRIVE_FEED_FORWARD_KV = 2.2757; //2.3854;
        public static final double DRIVE_FEED_FORWARD_KA = 0.39783; //0.15778;
        public static final double DRIVE_FEED_FORWARD_KS = 0.35123; //0.41036;

        public static final double TURN_PIDF0_P = 0.292;
        public static final double TURN_PIDF0_I = 0;
        public static final double TURN_PIDF0_D = 0;
        public static final double TURN_PIDF0_F = 0.0008;

        // -------------------------

        // -------------------------
        // SUPERSTRUCTURE
        // -------------------------

        public static final double ARM_PIDF0_P = 5.7799; //5.5363;
        public static final double ARM_PIDF0_I = 0;
        public static final double ARM_PIDF0_D = 0.78755; //0.10014;
        public static final double ARM_PIDF0_F = 0;

        public static final double ARM_PIDF1_P = 0.00032952;
        public static final double ARM_PIDF1_I = 0;
        public static final double ARM_PIDF1_D = 0;
        public static final double ARM_PIDF1_F = 0;

        public static final double ARM_FF0_KV = 3.4538;
        public static final double ARM_FF0_KS = 1.0671;
        public static final double ARM_FF0_KA = 0.50024;
        public static final double ARM_FF0_KG = 0.21272;

        public static final double INTAKE_PIDF0_P = 0.00076003;
        public static final double INTAKE_PIDF0_I = 0;
        public static final double INTAKE_PIDF0_D = 0;
        public static final double INTAKE_PIDF0_F = 0;

        public static final double INTAKE_PIDF1_P = 5.7351;
        public static final double INTAKE_PIDF1_I = 0;
        public static final double INTAKE_PIDF1_D = 0.29819;
        public static final double INTAKE_PIDF1_F = 0;

        public static final double INTAKE_FF0_KV = 4.19895;
        public static final double INTAKE_FF0_KS = 0.77761;
        public static final double INTAKE_FF0_KA = 2.03662;

        public static final double SHOOTER_PIDF0_P = 0.007518;
        public static final double SHOOTER_PIDF0_I = 0;
        public static final double SHOOTER_PIDF0_D = 0;
        public static final double SHOOTER_PIDF0_F = 0;

        public static final double SHOOTER_FF0_KV = 0.089526; //0.11151;
        public static final double SHOOTER_FF0_KS = 0.22492; //0.26382;
        public static final double SHOOTER_FF0_KA = 0.074204; //0.049897;

        // -------------------------

        public static final double THETA_PID_P = 0.05;
        public static final double THETA_PID_I = 0;
        public static final double THETA_PID_D = 0;
        public static final double THETA_MAX_DEG_S = 360;
        public static final double THETA_MAX_DEG_S2 = 720;

        public static final double ARM_MAX_RAD = Math.PI*23;
        public static final double ARM_MAX_RAD_2 = Math.PI*4;
    }

    public class PHYSICAL_CONSTANTS {

        public static final double TRACK_WIDTH = 24 - 2.625;
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH);
        public static final double TRACK_RADIUS_METERS = Math.sqrt((TRACK_WIDTH_METERS * TRACK_WIDTH_METERS) + (TRACK_WIDTH_METERS * TRACK_WIDTH_METERS));

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d((TRACK_WIDTH_METERS / 2.0), (TRACK_WIDTH_METERS / 2.0)),
            new Translation2d((TRACK_WIDTH_METERS / 2.0), -(TRACK_WIDTH_METERS / 2.0)),
            new Translation2d(-(TRACK_WIDTH_METERS / 2.0), (TRACK_WIDTH_METERS / 2.0)),
            new Translation2d(-(TRACK_WIDTH_METERS / 2.0), -(TRACK_WIDTH_METERS / 2.0))
        );

        public static final double DRIVE_WHEEL_DIAMETER_INCHES = 4; //Colsons: 3.905;

        public static final double DRIVE_WHEEL_DIAMETER_METERS = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER_INCHES);

        public static final double SHOOTER_WHEEL_DIAMETER_INCHES = 4;
        public static final double INTAKE_WHEEL_DIAMETER_INCHES = 1;

        public static final double SHOOTER_WHEEL_DIAMETER_FEET = SHOOTER_WHEEL_DIAMETER_INCHES / 12.0;
        public static final double INTAKE_WHEEL_DIAMETER_FEET = INTAKE_WHEEL_DIAMETER_INCHES / 12.0;

        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

        public static final double ARM_GEAR_RATIO = 253.333333;

        public static final double SHOOTER_GEAR_RATIO = 1.0;
        public static final double INTAKE_GEAR_RATIO = 2.333333;

        public static final double MAX_TRANSLATION_METERS = 4;
        public static final double MAX_ANGULAR_SPEED_RAD = Math.PI * 3;
        public static final double MAX_WHEEL_SPEED_METERS = 4.2;

        public static final Translation2d SPEAKER_POSITION = new Translation2d(0, 5.55);
        public static final Translation2d SHOT_TARGET_SPEAKER_POS = new Translation2d(0, 5.55);
        public static final double SPEAKER_HEIGHT_INCHES = 80.7505;
        public static final double SPEAKER_HEIGHT_METERS = Units.inchesToMeters(SPEAKER_HEIGHT_INCHES); 
        /* ARM PIVOT OF ROTATION */
        public static final double ARM_HEIGHT_INCHES = 15.5;
        public static final double ARM_HEIGHT_METERS = Units.inchesToMeters(ARM_HEIGHT_INCHES);
        public static final double ARM_LENGTH_INCHES = 22.4;
        public static final double ARM_LENGTH_METERS = Units.inchesToMeters(ARM_LENGTH_INCHES);

        public static final double ARM_SETPOINT_THRESHOLD = 0.01;
        public static final double ARM_ACCEL_SETPOINT_THRESHOLD = 0.0001;

        public static final double SHOOTER_SETPOINT_THRESHOLD = 20;
        public static final double SHOOTER_ACCEL_SETPOINT_THRESHOLD = 0.001;

        public static final double INTAKE_SETPOINT_THRESHOLD = 0.02;
        public static final double INTAKE_ACCEL_SETPOINT_THRESHOLD = 0.0004; 

        public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.223);

        public static final Translation3d SPEAKER_TOP_RIGHT =
            new Translation3d(
                Units.inchesToMeters(18.055),
                Units.inchesToMeters(238.815),
                Units.inchesToMeters(83.091));

        public static final Translation3d SPEAKER_TOP_LEFT =
            new Translation3d(
                Units.inchesToMeters(18.055),
                Units.inchesToMeters(197.765),
                Units.inchesToMeters(83.091));

        public static final Translation3d SPEAKER_BUTTOM_RIGHT =
            new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
        public static final Translation3d SPEAKER_BUTTOM_LEFT =
            new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

        /** Center of the speaker opening (blue alliance) */
        public static final Translation3d CENTRE_SPEAKER_OPENING =
            SPEAKER_BUTTOM_LEFT.interpolate(SPEAKER_TOP_RIGHT, 0.5);

        public static final Pose3d left_photon = new Pose3d(
            Units.inchesToMeters(0.2714625), 
            Units.inchesToMeters(0.2714625), 
            Units.inchesToMeters(0), 
            new Rotation3d(
                0, 
                30, 
                -10
            )
        );

        public static final Pose3d right_photon = new Pose3d(
            Units.inchesToMeters(0.2714625), 
            Units.inchesToMeters(-0.2714625), 
            Units.inchesToMeters(0), 
            new Rotation3d(
                0, 
                30, 
                10
            )
        );

        // ---------------------
        // MOTORS
        // ---------------------

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int TURN_CURRENT_LIMIT = 30;

        public static final int ARM_CURRENT_LIMIT = 60;

        public static final int INTAKE_CURRENT_LIMIT = 40;
        public static final int SHOOTER_CURRENT_LIMIT = 50;

        public static final double NOMINAL_VOLTAGE = 12;

        public static final double DRIVE_LOOP_RAMP_RATE = 0.2;
        public static final double TURN_LOOP_RAMP_RATE = 0.1;

        // --------------------

        // --------------------
        // ENCODER OFFSETS
        // --------------------

        public static final double FRONT_LEFT_OFFSET = -0.425;
        public static final double FRONT_RIGHT_OFFSET = -0.2419;
        public static final double REAR_LEFT_OFFSET = 0.1791;
        public static final double REAR_RIGHT_OFFSET = 0.2607;
        
        public static final double ARM_OFFSET = 1.9;
        

        // --------------------

        public static final double ARM_INTAKE_POSITION = -0.07;
        public static final double ARM_POSITION_HIGH = 1;
        public static final double ARM_HOLD_POSITION_LOw = 0.161;
        public static final double ARM_AMP_POSITION = 1.49;
        public static final double ARM_POSITION_PREPARE_CLIMB = 1.3;

        public static final double GYRO_REVERSED = -1;

        public static final double GYRO_OFFSET = 180;
    }

    public static final double LOOP_TIME_S = 0.02;
    public static final int LOOP_TIME_MS = 20;

    public static final double INTAKE_DEBOUNCE_TIME = 0.3;
}