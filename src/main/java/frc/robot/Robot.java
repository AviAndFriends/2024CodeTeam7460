// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.SensorConstants;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Ultrasonic m_rangeFinder = new Ultrasonic(1, 2);
  double[] uSonicValueHistory = new double[SensorConstants.LOAD_LENGTH];

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.

    m_robotContainer = new RobotContainer();

    Shuffleboard.getTab("Sensors").add(m_rangeFinder); // should this be here? idk

    m_rangeFinder.setAutomaticMode(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    Logger.recordOutput("Ring loaded", isLoaded());
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  public boolean isLoaded() {
    if (m_rangeFinder.isRangeValid()) {
      // We can read the distance in millimeters
      double distanceMillimeters = m_rangeFinder.getRangeMM();
      // ... or in inches
      double distanceInches = m_rangeFinder.getRangeInches();

      Logger.recordOutput("Ultrasonic Distance[mm]", distanceMillimeters);

      // We can also publish the data itself periodically
      SmartDashboard.putNumber("Distance[mm]", distanceMillimeters);
      SmartDashboard.putNumber("Distance[inch]", distanceInches);

      
      for(int i = 0; i < SensorConstants.LOAD_LENGTH - 1; i++) {
        uSonicValueHistory[i] = uSonicValueHistory[i + 1];
      }

      uSonicValueHistory[SensorConstants.LOAD_LENGTH - 1] = distanceMillimeters;

      String histString = "";
      for(double element : uSonicValueHistory) {
        histString += String.valueOf(element) + " | ";
      }

      System.out.println(histString);

      for(double element : uSonicValueHistory) {
        if(element > SensorConstants.LOAD_DISTANCE) {
          return false;
        }
      }

      return true;
    }

    return false;
  }
}
