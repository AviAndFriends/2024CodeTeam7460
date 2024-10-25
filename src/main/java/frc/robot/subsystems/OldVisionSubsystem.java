package frc.robot.subsystems;

//Mo added import
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

//End of Mo added import?
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OldVisionSubsystem extends SubsystemBase {
    // It says we gotta match the name of the camera but I'm not sure if this
    // matches "enough"
    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    double xPosition;
    double yPosition;

    public OldVisionSubsystem() {

    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        // Logger.recordOutput("Targets seen", result.hasTargets());

        // Mo's additional Stuff from here forwards

        boolean hasTargets = result.hasTargets();

        // Get a list of currently tracked targets
        // There might be an issue with scope here
        if (hasTargets) {
            // Since it's possible we can see multiple targerts, like on the speaker
            // We have every aprilTag data on a list
            List<PhotonTrackedTarget> targets = result.getTargets();
            for (PhotonTrackedTarget currentTarget : targets) {
                if (currentTarget.getFiducialId() == 7) {
                    Transform3d relativePosition = currentTarget.getBestCameraToTarget();
                    yPosition = relativePosition.getY();
                    xPosition = relativePosition.getX();
                }

            }
            // This is just the best most accurate one on the list (not sure which metric
            // determined that)
            // PhotonTrackedTarget target = result.getBestTarget();

            // these last two are acting up on me so I changed it from .getCorners to
            // .getDetectedCorners
            // and I'm commenting out: Transform2d pose = target.getCameraToTarget();
            // List<TargetCorner> corners = target.getDetectedCorners();

            // A check to make sure nothing wrong is happening

            // int targetID = target.getFiducialId();
            // double poseAmbiguity = target.getPoseAmbiguity();
            // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

            // //Check #2

            // System.out.println(targetID);
            // System.out.println(poseAmbiguity);
            // System.out.println(bestCameraToTarget);
            // System.out.println(alternateCameraToTarget);
        }

        Logger.recordOutput("X Position from april tag", xPosition);
        Logger.recordOutput("Y Position from April Tag", yPosition);

        /*
         * Here's debugging stuff
         * //Capture pre-process camera stream image
         * camera.takeInputSnapshot();
         * 
         * //Capture post-process camera stream image
         * camera.takeOutputSnapshot();
         */
        // I litterally do not know what this even does or where to place it.

    }

    // THIS IS STUFF i COPY AND PASTED FROM GITHUB, THIS STUFF BELONGS IN COMMANDS
    // BECAUSE THATS WHAT THEY ARE

    // Constants such as camera and target height stored. Change per robot and goal!
    // final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(7.2);
    // final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // // Angle between horizontal and the camera.
    // final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(15);

    // // How far from the target we want to be
    // final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // // I commentened the line of code below because it's already present above
    // // PhotonCamera camera = new PhotonCamera("photonvision");

    // // PID constants should be tuned per robot
    // final double LINEAR_P = 0.1;
    // final double LINEAR_D = 0.0;
    // PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    // final double ANGULAR_P = 0.1;
    // final double ANGULAR_D = 0.0;
    // PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    // XboxController xboxController = new XboxController(0);

    // // Drive motors WE NEED TO CHANGE THIS SO IT WORKS WITH OUR MOTORS
    // PWMVictorSPX leftMotor = new PWMVictorSPX(0);
    // PWMVictorSPX rightMotor = new PWMVictorSPX(1);
    // DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    // // @Override is causing issues so Imma comment it out
    // public void teleopPeriodic1() { // bad name, change
    //     double forwardSpeed;
    //     double rotationSpeed;
    //     // Essentially, this is a TURN TO TARGET command
    //     if (xboxController.getAButton()) {
    //         // Vision-alignment mode
    //         // Query the latest result from PhotonVision
    //         var result = camera.getLatestResult();

    //         if (result.hasTargets()) {
    //             // First calculate range
    //             double range = PhotonUtils.calculateDistanceToTargetMeters(
    //                     CAMERA_HEIGHT_METERS,
    //                     TARGET_HEIGHT_METERS,
    //                     CAMERA_PITCH_RADIANS,
    //                     Units.degreesToRadians(result.getBestTarget().getPitch()));

    //             // Use this range as the measurement we give to the PID controller.
    //             // -1.0 required to ensure positive PID controller effort _increases_ range
    //             forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

    //             // Also calculate angular power
    //             // -1.0 required to ensure positive PID controller effort _increases_ yaw
    //             rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    //         } else {
    //             // If we have no targets, stay still.
    //             forwardSpeed = 0;
    //             rotationSpeed = 0;
    //         }
    //     } else {
    //         // Manual Driver Mode
    //         forwardSpeed = -xboxController.getRightY();
    //         rotationSpeed = xboxController.getLeftX();
    //     }

    //     // Use our forward/turn speeds to control the drivetrain
    //     // THERE'S AN ERROR IN THE LINE BELOW THIS, IDK HOW TO FIX
    //     drive.arcadeDrive(forwardSpeed, rotationSpeed);
    // }
    // // Essentially, new copy and paste , this is a AIM SHOOTER AT TARGET command

    // // @Override is causing issues so Imma comment it out
    // public void teleopPeriodic2() {// bad name
    //     double forwardSpeed;
    //     double rotationSpeed;

    //     forwardSpeed = -xboxController.getRightY();

    //     if (xboxController.getAButton()) {
    //         // Vision-alignment mode
    //         // Query the latest result from PhotonVision
    //         var result = camera.getLatestResult();

    //         if (result.hasTargets()) {
    //             // Calculate angular turn power
    //             // -1.0 required to ensure positive PID controller effort _increases_ yaw
    //             rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    //         } else {
    //             // If we have no targets, stay still.
    //             rotationSpeed = 0;
    //         }
    //     } else {
    //         // Manual Driver Mode
    //         rotationSpeed = xboxController.getLeftX();
    //     }

    //     // Use our forward/turn speeds to control the drivetrain
    //     drive.arcadeDrive(forwardSpeed, rotationSpeed);
    // }
    // // Essentially, this is a MOVE ROBOT TO TARGET SO WE CAN ACTUALLY SHOOT

    // // PID constants should be tuned per robot
    // final double P_GAIN = 0.1;
    // final double D_GAIN = 0.0;
    // PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

    // /*
    //  * this is having some issues so I just commented it out
    //  * 
    //  * @Override
    //  * public void robotInit() {
    //  * xboxController = new XboxController(0);
    //  * }
    //  */

    // // @Override is causing issues so Imma comment it out
    // public void teleopPeriodic3() { // bad name
    //     double forwardSpeed;
    //     double rotationSpeed = xboxController.getLeftX();

    //     if (xboxController.getAButton()) {
    //         // Vision-alignment mode
    //         // Query the latest result from PhotonVision
    //         var result = camera.getLatestResult();

    //         if (result.hasTargets()) {
    //             // First calculate range
    //             double range = PhotonUtils.calculateDistanceToTargetMeters(
    //                     CAMERA_HEIGHT_METERS,
    //                     TARGET_HEIGHT_METERS,
    //                     CAMERA_PITCH_RADIANS,
    //                     Units.degreesToRadians(result.getBestTarget().getPitch()));

    //             // Use this range as the measurement we give to the PID controller.
    //             // -1.0 required to ensure positive PID controller effort _increases_ range
    //             forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);
    //         } else {
    //             // If we have no targets, stay still.
    //             forwardSpeed = 0;
    //         }
    //     } else {
    //         // Manual Driver Mode
    //         forwardSpeed = -xboxController.getRightY();
    //     }

    //     // Use our forward/turn speeds to control the drivetrain
    //     drive.arcadeDrive(forwardSpeed, rotationSpeed);
    // }
}
