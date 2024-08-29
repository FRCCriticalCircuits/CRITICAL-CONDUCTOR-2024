package frc.team9062.robot.Subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private PhotonCamera left_camera, right_camera;
    private List<Pose2d> samples;
    private Pose2d averageEstimate;
    
    public Vision() {
        left_camera = new PhotonCamera("left_photon");
        right_camera = new PhotonCamera("right_photon");
    }

    public void refreshSamples() {
        samples.clear();

        for(PhotonTrackedTarget target : left_camera.getLatestResult().targets) {
            Pose3d estimate = PhotonUtils.estimateFieldToRobotAprilTag(
                null, 
                null, 
                null
            );

            samples.add(estimate.toPose2d());
        }

        for(PhotonTrackedTarget target : right_camera.getLatestResult().targets) {
            Pose3d estimate = PhotonUtils.estimateFieldToRobotAprilTag(
                null, 
                null, 
                null
            );

            samples.add(estimate.toPose2d());
        }
    }

    public void getAverageEstimate() {
        Translation2d av_translation = new Translation2d(0, 0);
        double rot_av = 0;

        for(Pose2d sample : samples) {
            av_translation.plus(sample.getTranslation());
            rot_av += sample.getRotation().getDegrees();
        }

        av_translation = av_translation.div(samples.size());
        rot_av = rot_av / samples.size();

        averageEstimate = new Pose2d(av_translation.getX(), av_translation.getY(), Rotation2d.fromDegrees(rot_av));
    }

    public Pose2d estimatePose() {
        return new Pose2d(); 
    }

    @Override
    public void periodic() {}
}