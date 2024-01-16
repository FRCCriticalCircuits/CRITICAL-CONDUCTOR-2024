package frc.team9062.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class fucidialTag {
    private int id;
    private Pose2d pose2d;
    private Pose3d pose3d;

    public fucidialTag(int id, Pose2d pose2d) {
        this.id = id;
        this.pose2d = pose2d;
    }

    public fucidialTag(int id, Pose3d pose3d) {
        this.id = id;
        this.pose3d = pose3d;
    }

    public fucidialTag(int id, Pose2d pose2d, Pose3d pose3d) {
        this.id = id;
        this.pose2d = pose2d;
        this.pose3d = pose3d;
    }

    public int getID() {
        return id;
    }

    public Pose2d getPose2d() {
        return pose2d;
    }

    public Pose3d getPose3d() {
        return pose3d;
    }
}
