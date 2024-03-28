package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class SensorConstants {
        // Constants for sensors (ie. ultrasonic sensor)

        // Ultrasonic constants
        public static final int LOAD_LENGTH = 3; // # of cycles to be considered loaded
        public static final double LOAD_DISTANCE = 70; // mm away from sensor to be considered loaded

        // Vision constants
        public static final String CAMERA_NAME = "photonvision";
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
        public static final double xSpeed = 2.4; //change max speed
        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)
    
        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot

        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    
        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 7;
        public static final int kRearLeftDrivingCanId = 5;
        public static final int kFrontRightDrivingCanId = 1;
        public static final int kRearRightDrivingCanId = 3;
    
        public static final int kFrontLeftTurningCanId = 8;
        public static final int kRearLeftTurningCanId = 6;
        public static final int kFrontRightTurningCanId = 2;
        public static final int kRearRightTurningCanId = 4;
    
        public static final boolean kGyroReversed = false;
        public static final int kTeleDriveMaxAngularSpeedRadiansPerSecond = 0;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 0;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 0;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0;
    
      }
    
      public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
    
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
        // bevel pinion
        public static final double kDrivingMotorReduction =
            (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps =
            (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
    
        public static final double kDrivingEncoderPositionFactor =
            (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor =
            ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second
    
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor =
            (2 * Math.PI) / 60.0; // radians per second
    
        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput =
            kTurningEncoderPositionFactor; // radians
    
        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;
    
        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
    
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
       // public static final double kTeleDriveMaxSpeedMetersPerSecond = 0;    
      }
    
      public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
        public static final double kDeadband = 0;
        public static final int SUPPLEMENTAL_CONTROLLER_PORT = 1;
        public static final int BUTTON_BOX_PORT = 2;
      }
    
      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 0.7;
        public static final double kPYController = 0.7;
        public static final double kPThetaController = 10;
    
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
    
      public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
      }

      public static final class AssemblyConstants {
        public static final int ASSEMBLY_MOTOR_ID = 13;
        //originally at 0.3
        public static final double ASSEMBLY_MAX_SPEED = 0.5;
          public static final double ASSEMBLY_SHOOTING_ANGLE = 0.411 + 0; //changed from 6.3
        // public static final double ASSEMBLY_NEW_SHOOTING_ANGLE = 3.5 //changed fom 3
         // preferred number by driver (caleb said so)

         public static final double ASSEMBLY_NEW_SHOOTING_ANGLE = 0.411 + 0.043; //changed fom 3
        //  public static final double ASSEMBLY_INTAKE_ANGLE = 0.2;

        public static final double ASSEMBLY_INTAKE_ANGLE = 0.411 + 0.001;
        // public static final double ASSEMBLY_PASS_ANGLE = 7;
        public static final double ASSEMBLY_PASS_ANGLE = 0.411 + 0.101;

        //Add SmartMotion PID Constants Code from RevRobotics
         public static final double ASSEMBLY_AMP_SHOOT = 0.411 + 0; //32.9
         public static final double ASSEMBLY_AUTONOMOUS1_SHOOTING_ANGLE = 0.411 + 0; //5.9
      }
//
      public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 9;
        public static final double INTAKE_MAX_SPEED = -0.3;
      }


      public static final class SlideConstants {
        public static final int SLIDE_MOTOR_ID = 12;
        //originally at 0.8
        public static final double SLIDE_MAX_SPEED = .9;
        public static final double SLIDE_LOW = 0.1;
        public static final double SLIDE_HIGH = 5;
        public static final double SLIDE_AMP_SHOOT = 9.5;  
      }

      public static final class BeltConstants {
        public static final int BELT_MOTOR_ID = 10;
        public static final double BELT_MAX_SPEED = 0.9;
        public static final double BELT_BACKUP_OFFEST = -0.25;
      }

      public static final class FlyWheelConstants {
        public static final int FW_MOTOR_ID = 11;
        public static final double FW_MAX_SPEED = -0.7;
        // public static final double FW_MAX_SPEED = -0;

      }
      
    public static int currentMode;
}
