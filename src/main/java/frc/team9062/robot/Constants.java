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

        public static Port GYRO_PORT = Port.kUSB;

        public static final int GAMEPAD_DRIVER = 0;
        public static final int GAMEPAD_OPERATOR = 1;

    }

    public class TUNED_CONSTANTS {

        // -------------------------
        // DRIVEBASE
        // -------------------------

        public static final double DRIVE_PID0_P = 4.497E-06;
        public static final double DRIVE_PID0_I = 0; 
        public static final double DRIVE_PID0_D = 0;

        public static final double DRIVE_FEED_FORWARD_KV = 2.3854; // todo: characterize drive
        public static final double DRIVE_FEED_FORWARD_KA = 0.15778;
        public static final double DRIVE_FEED_FORWARD_KS = 0.41036;

        public static final double TURN_PID0_P = 0.2922;
        public static final double TURN_PID0_I = 0;
        public static final double TURN_PID0_D = 0.0008;

        // -------------------------

    }

    public class PHYSICAL_CONSTANTS {

        public static final double TRACK_WIDTH = 24 - 2.625;
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH);

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d((TRACK_WIDTH_METERS / 2.0), (TRACK_WIDTH_METERS / 2.0)),
            new Translation2d((TRACK_WIDTH_METERS / 2.0), -(TRACK_WIDTH_METERS / 2.0)),
            new Translation2d(-(TRACK_WIDTH_METERS / 2.0), (TRACK_WIDTH_METERS / 2.0)),
            new Translation2d(-(TRACK_WIDTH_METERS / 2.0), -(TRACK_WIDTH_METERS / 2.0))
        );

        public static final double DRIVE_WHEEL_DIAMETER_INCHES = 3.963;
        public static final double DRIVE_WHEEL_DIAMTER_METERS = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER_INCHES);

        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

        public static final double MAX_TRANSLATION_METERS = 4.3;
        public static final double MAX_ANGULAR_SPEED_RAD = Math.PI;
        public static final double MAX_WHEEL_SPEED_METERS = 4.8; 

        // ---------------------
        // MOTORS
        // ---------------------

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int TURN_CURRENT_LIMIT = 30;

        public static final double NOMINAL_VOLTAGE = 12;

        public static final double DRIVE_LOOP_RAMP_RATE = 0.2;
        public static final double TURN_LOOP_RAMP_RATE = 0.2;

        // --------------------

        // --------------------
        // ENCODER OFFSETS
        // --------------------

        public static final double FRONT_LEFT_OFFSET = -152.9;
        public static final double FRONT_RIGHT_OFFSET = -87;
        public static final double REAR_LEFT_OFFSET = 62.3;
        public static final double REAR_RIGHT_OFFSET = 96.1;

        // --------------------

    }

    public static final double LOOP_TIME_S = 0.02;
    public static final int LOOP_TIME_MS = 20;
}
