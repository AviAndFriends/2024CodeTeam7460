package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("photonvision");
    

    public VisionSubsystem() {
        
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        Logger.recordOutput("Targets seen", result.hasTargets());
    }
}
