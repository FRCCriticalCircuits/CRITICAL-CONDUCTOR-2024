package frc.team9062.robot.Util;
/**
 *  LimelightHelpers Interface
 */

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team9062.robot.Util.lib.LimelightHelpers;

public class Limelight {
    private static Limelight instance;
    private String llname;
    private NetworkTable LLTable;
    private final Translation2d target = new Translation2d(0, 5.55);

    public Limelight(String llname) {
        this.llname = llname;

        LLTable = NetworkTableInstance.getDefault().getTable(llname);
    }

    public static Limelight getInstance(String llname) {
        if (instance == null) {
            instance = new Limelight(llname);
        }

        return instance;
    }

    public double get_tx() {
        return LimelightHelpers.getTX(llname);
    }
    public double get_ty() {
        return LimelightHelpers.getTY(llname);
    }

    public Pose2d getBotPose2d() {
        return LimelightHelpers.getBotPose2d(llname);
    }

    public Pose3d getBotPose3d(){
        return LimelightHelpers.getBotPose3d(llname);
    }

    public Rotation2d getHeadingToTarget(Supplier<Pose2d> pose) {
        Translation2d position = pose.get().getTranslation();
        Translation2d distances = position.minus(target);

        return distances.getAngle();
    }

    /* PASTED FROM LIMELIGHTHELPERS */
    public static Pose2d toPose2D(double[] inData){
        if(inData.length < 6)
        {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    /* PASTED FROM LIMELIGHTHELPERS */
    public static Pose3d toPose3D(double[] inData){
        if(inData.length < 6)
        {
            System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }
}