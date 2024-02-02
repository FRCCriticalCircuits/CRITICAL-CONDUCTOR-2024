package frc.team9062.robot;

import edu.wpi.first.math.geometry.Translation2d;
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

        public static final int SHOOTER_ID = 17;
        public static final int SHOOTER_FOLLOWER_ID = 18;

        public static final int ROLLER_ID = 20;

        // -------------------------------------------------


        public static Port GYRO_PORT = Port.kUSB;

        public static final int GAMEPAD_DRIVER = 0;
        public static final int GAMEPAD_OPERATOR = 1;

    }

    public class TUNED_CONSTANTS {

        // -------------------------
        // DRIVEBASE
        // -------------------------

        public static final double DRIVE_PIDF0_P = 4.497E-06;
        public static final double DRIVE_PIDF0_I = 0; 
        public static final double DRIVE_PIDF0_D = 0;
        public static final double DRIVE_PIDF0_F = 0;

        public static final double DRIVE_FEED_FORWARD_KV = 2.3854; // todo: re-characterize drive when top half is assembled
        public static final double DRIVE_FEED_FORWARD_KA = 0.15778;
        public static final double DRIVE_FEED_FORWARD_KS = 0.41036;

        public static final double TURN_PIDF0_P = 0.292;
        public static final double TURN_PIDF0_I = 0;
        public static final double TURN_PIDF0_D = 0;
        public static final double TURN_PIDF0_F = 0.0008;

        // -------------------------

        // -------------------------
        // SUPERSTRUCTURE
        // -------------------------

        public static final double ARM_PIDF0_P = 1;
        public static final double ARM_PIDF0_I = 0;
        public static final double ARM_PIDF0_D = 0;
        public static final double ARM_PIDF0_F = 0.1;

        public static final double ARM_PIDF1_P = 0;
        public static final double ARM_PIDF1_I = 0;
        public static final double ARM_PIDF1_D = 0;
        public static final double ARM_PIDF1_F = 0;

        public static final double ARM_FF0_KV = 0;
        public static final double ARM_FF0_KS = 0;
        public static final double ARM_FF0_KA = 0;
        public static final double ARM_FF0_KG = 0;

        public static final double INTAKE_PIDF0_P = 0;
        public static final double INTAKE_PIDF0_I = 0;
        public static final double INTAKE_PIDF0_D = 0;
        public static final double INTAKE_PIDF0_F = 0;

        public static final double SHOOTER_PIDF0_P = 0;
        public static final double SHOOTER_PIDF0_I = 0;
        public static final double SHOOTER_PIDF0_D = 0;
        public static final double SHOOTER_PIDF0_F = 0;

        public static final double SHOOTER_FF0_KV = 0;
        public static final double SHOOTER_FF0_KS = 0;
        public static final double SHOOTER_FF0_KA = 0;

        // -------------------------

        public static final double THETA_PID_P = 0;
        public static final double THETA_PID_I = 0;
        public static final double THETA_PID_D = 0;
        public static final double THETA_MAX_DEG_S = Math.PI;
        public static final double THETA_MAX_DEG_S2 = Math.PI*2;

        public static final double ARM_MAX_RAD = Math.PI*6;
        public static final double ARM_MAX_RAD_2 = Math.PI*8;
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

        public static final double DRIVE_WHEEL_DIAMETER_INCHES = 3.963;
        public static final double DRIVE_WHEEL_DIAMETER_METERS = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER_INCHES);

        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

        public static final double ARM_GEAR_RATIO = 250.0;

        public static final double MAX_TRANSLATION_METERS = 4.3;
        public static final double MAX_ANGULAR_SPEED_RAD = Math.PI * 2;
        public static final double MAX_WHEEL_SPEED_METERS = 4.8; 

        // ---------------------
        // MOTORS
        // ---------------------

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int TURN_CURRENT_LIMIT = 30;

        public static final int ARM_CURRENT_LIMIT = 60;

        public static final double NOMINAL_VOLTAGE = 12;

        public static final double DRIVE_LOOP_RAMP_RATE = 0.2;
        public static final double TURN_LOOP_RAMP_RATE = 0.2;

        // --------------------

        // --------------------
        // ENCODER OFFSETS
        // --------------------

        public static final double FRONT_LEFT_OFFSET = -0.4318;
        public static final double FRONT_RIGHT_OFFSET = -0.2419;
        public static final double REAR_LEFT_OFFSET = 0.1791;
        public static final double REAR_RIGHT_OFFSET = 0.2607;

        // --------------------

        public static final double ARM_INTAKE_POSITION = -0.08;
        public static final double ARM_HOLD_POSITION = -1.297;

        public static final double GYRO_REVERSED = -1;

        public static final double GYRO_OFFSET = 180;
    }

    public static final double LOOP_TIME_S = 0.02;
    public static final int LOOP_TIME_MS = 20;
}