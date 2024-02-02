package frc.team9062.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
    private static Rollers instance;
    

    public Rollers() {

    }

    public static Rollers getInstance() {
        if (instance == null) {
            instance = new Rollers();
        }

        return instance;
    }
}