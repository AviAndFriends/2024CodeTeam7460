package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    public VisionSubsystem() {

    }

    @Override
    public void periodic() {
        Logger.recordOutput("XPos from April Tag", getXPosition(7));
        Logger.recordOutput("YPos fom april tag", getYPosition(7));
    }

    public double getXPosition(int targetID) {
        var result = camera.getLatestResult();

        boolean hasTargets = result.hasTargets();

        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            for (PhotonTrackedTarget currentTarget : targets) {
                if (currentTarget.getFiducialId() == targetID) {
                    Transform3d relativePosition = currentTarget.getBestCameraToTarget();
                    return (relativePosition.getX());
                }

            }
            
        }
        return 10000;
    }

    public double getYPosition(int targetID) {
        var result = camera.getLatestResult();

        boolean hasTargets = result.hasTargets();

        if (hasTargets) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            for (PhotonTrackedTarget currentTarget : targets) {
                if (currentTarget.getFiducialId() == targetID) {
                    Transform3d relativePosition = currentTarget.getBestCameraToTarget();
                    return (relativePosition.getY());
                }

            }

            
        }
        return 10000;

    }

}
