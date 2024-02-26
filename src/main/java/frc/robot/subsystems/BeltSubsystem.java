package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class BeltSubsystem {
    public class Robot extends TimedRobot {
    private XboxController b_Controller;
    private static final int deviceID = 1;
    private CANSparkMax b_motor;
    private SparkPIDController b_pidController;
    private RelativeEncoder b_encoder;
    public double b_kP, b_kI, b_kD, b_kIz, b_kFF, b_kMaxOutput, b_kMinOutput, b_maxRPM;

    @Override
    public void robotInit() {
      b_Controller = new XboxController(0);

      // initialize motor
      b_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
      /**
       * The RestoreFactoryDefaults method can be used to reset the configuration parameters in the
       * SPARK MAX to their factory default state. If no argument is passed, these parameters will
       * not persist between power cycles
       */
      b_motor.restoreFactoryDefaults();

      /**
       * In order to use PID functionality for a controller, a SparkPIDController object is
       * constructed by calling the getPIDController() method on an existing CANSparkMax object
       */
      b_pidController = b_motor.getPIDController();

      // Encoder object created to display position values
      b_encoder = b_motor.getEncoder();

      // PID coefficients
      b_kP = 6e-5;
      b_kI = 0;
      b_kD = 0;
      b_kIz = 0;
      b_kFF = 0.000015;
      b_kMaxOutput = 1;
      b_kMinOutput = -1;
      b_maxRPM = 5700;

      // set PID coefficients
      b_pidController.setP(b_kP);
      b_pidController.setI(b_kI);
      b_pidController.setD(b_kD);
      b_pidController.setIZone(b_kIz);
      b_pidController.setFF(b_kFF);
      b_pidController.setOutputRange(b_kMinOutput, b_kMaxOutput);

      /*// display PID coefficients on SmartDashboard
          SmartDashboard.putNumber("P Gain", kP);
          SmartDashboard.putNumber("I Gain", kI);
          SmartDashboard.putNumber("D Gain", kD);
          SmartDashboard.putNumber("I Zone", kIz);
          SmartDashboard.putNumber("Feed Forward", kFF);
          SmartDashboard.putNumber("Max Output", kMaxOutput);
          SmartDashboard.putNumber("Min Output", kMinOutput);
      */
    }

    @Override
    public void teleopPeriodic() {
      // read PID coefficients from SmartDashboard
      /*
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
      */
      /*
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { m_pidController.setP(p); kP = p; }
      if((i != kI)) { m_pidController.setI(i); kI = i; }
      if((d != kD)) { m_pidController.setD(d); kD = d; }
      if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) {
        m_pidController.setOutputRange(min, max);
        kMinOutput = min; kMaxOutput = max;
      }


       * PIDController objects are commanded to a set point using the
       * SetReference() method.
       *
       * The first parameter is the value of the set point, whose units vary
       * depending on the control type set in the second parameter.
       *
       * The second parameter is the control type can be set to one of four
       * parameters:
       *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
       *  com.revrobotics.CANSparkMax.ControlType.kPosition
       *  com.revrobotics.CANSparkMax.ControlType.kVelocity
       *  com.revrobotics.CANSparkMax.ControlType.kVoltage
       */
      // double b_setPoint = b_Controller.getLeftY()*b_maxRPM;
      // b_pidController.setReference(b_setPoint, CANSparkMax.ControlType.kVelocity);

      // SmartDashboard.putNumber("SetPoint", setPoint);
      // SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    }
  }
}
