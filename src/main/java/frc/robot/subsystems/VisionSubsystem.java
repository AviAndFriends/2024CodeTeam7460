package frc.robot.subsystems;

//Mo added import
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

//End of Mo added import?
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
        if(getYPosition(7) != 10000) {
            SmartDashboard.putBoolean("Tag 7 Visible", true);
        } else {
            SmartDashboard.putBoolean("Tag 7 Visible", false);
        }


        if(getYPosition(4) != 10000) {
            SmartDashboard.putBoolean("Tag 4 Visible", true);
        } else {
            SmartDashboard.putBoolean("Tag 4 Visible", false);
        }
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
