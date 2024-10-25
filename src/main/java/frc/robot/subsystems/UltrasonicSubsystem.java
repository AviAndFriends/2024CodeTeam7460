package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants;

public class UltrasonicSubsystem extends SubsystemBase{
    Ultrasonic m_rangeFinder = new Ultrasonic(2, 1);
    double[] uSonicValueHistory = new double[SensorConstants.LOAD_LENGTH];
    public boolean loaded;
    

    public UltrasonicSubsystem() {
        // Shuffleboard.getTab("Sensors").add(m_rangeFinder); // should this be here? idk
        m_rangeFinder.setAutomaticMode(true);
    }

    @Override
    public void periodic() {
        loaded = isLoaded();
        Logger.recordOutput("Ring loaded", loaded);
        Logger.recordOutput("Ultrasonic Distance", m_rangeFinder.getRangeMM());

    }


    private boolean isLoaded() {
    if (m_rangeFinder.isRangeValid()) {
      // We can read the distance in millimeters
      double distanceMillimeters = m_rangeFinder.getRangeMM();
      // ... or in inches
      double distanceInches = m_rangeFinder.getRangeInches();

      Logger.recordOutput("Ultrasonic Distance[mm]", distanceMillimeters);

    
      
      for(int i = 0; i < SensorConstants.LOAD_LENGTH - 1; i++) {
        uSonicValueHistory[i] = uSonicValueHistory[i + 1];
      }

      uSonicValueHistory[SensorConstants.LOAD_LENGTH - 1] = distanceMillimeters;

      String histString = "";
      for(double element : uSonicValueHistory) {
        histString += String.valueOf(element) + " | ";
      }

      // System.out.println(histString);

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
