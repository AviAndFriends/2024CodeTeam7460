/* 
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class SwerveXboxCmd extends Command {
    
  private final DriveSubsystem driveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveXboxCmd(
      DriveSubsystem driveSubsystem,
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    this.driveSubsystem = driveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    /*Get real time joystick inputs */
    /* 
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    /*Applies Deadzone/deadband */
    /* 
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

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
    if (fieldOrientedFunction.get()) {
      /*relative to field */
      /* 
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, turningSpeed, driveSubsystem.getRotation2d());
    } else {
      /*relative to robot */
      /* 
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    /*convert chassis speeds to individual module states */
    /* SwerveModuleState[] moduleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    /*output each module states to wheels */
    /* 
    driveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
*/