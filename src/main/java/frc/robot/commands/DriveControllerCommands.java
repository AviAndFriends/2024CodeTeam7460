/* 
package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveControllerCommands extends Command {
    private static final Rotation2d fieldRelative = null;
    private static final String rateLimit = null;
    int d_controllerPort = 0;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter; 

  /* public SwerveXboxCmd(SwerveSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction) {
      this.swerveSubsystem = swerveSubsystem;
      this.xSpdFunction = xSpdFunction;
      this.ySpdFunction = ySpdFunction;
      this.fieldOrientedFunction = fieldOrientedFunction;
      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      addRequirements(swerveSubsystem);
  }*/
/* 
  @Override
  public void initialize() {
    XboxController d_controller = new XboxController(d_controllerPort);
    int xSpeed = 0;
    int ySpeed = 0;
    int rot = 0;
    boolean fieldOriented = true;
    boolean slowMode = false;
  }

  @Override
  public void execute() {
    XboxController d_controller;
    double xSpeed = d_controller.getLeftX();
    double ySpeed = d_controller.getLeftY();
    double rot = d_controller.getRightX();
    boolean fieldOriented = !d_controller.getLeftBumper();
    boolean slowMode = d_controller.getRightBumper();

    // Applies slow mode
    if (slowMode) {
      xSpeed = 0.3 * xSpeed;
      ySpeed = 0.3 * ySpeed;
      rot = 0.3 * rot;
    }

    /*Applies Deadzone/deadband */
    /* 
    if (xSpeed < 0.05) {
      xSpeed = 0;
    }
    if (ySpeed < 0.05) {
      ySpeed = 0;
    }
    if (rot < 0.05) {
      rot = 0;
    }

    /*Make the Driving smoother */
    /* 
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed =
        turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    /* Construct desired chassis speeds */
    /* 
    ChassisSpeeds chassisSpeeds;
    if (fieldOriented.get) {
      /*relative to field */
      /* 
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
    } else {
      /*relative to robot */
      /* 
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
    }

    /*convert chassis speeds to individual module states */
    /* 
    SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    /*output each module states to wheels */
    /* 
    swerveSubsystem.setModuleStates(moduleStates);

    // D-pad commands

  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
*/