package frc.team9062.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;

public class AimData {
    public Rotation2d heading;
    public double distance;
    
    public AimData(Rotation2d heading, double distance) {
        this.heading = heading;
        this.distance = distance;
    }
}
