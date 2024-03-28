package frc.robot.subsystems;

import javax.script.AbstractScriptEngine;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AssemblyConstants;

public class AssemblySubsystem extends SubsystemBase {
    private final CANSparkMax a_motor = new CANSparkMax(AssemblyConstants.ASSEMBLY_MOTOR_ID,
            CANSparkLowLevel.MotorType.kBrushless);

    private RelativeEncoder a_encoder;
    private SparkPIDController a_pidController;
    private AbsoluteEncoder a_absEncoder;
    public double a_kP, a_kI, a_kD, a_kIz, a_kFF, a_kMaxOutput, a_kMinOutput, a_maxRPM;

    public AssemblySubsystem() {

        a_motor.restoreFactoryDefaults();
        a_pidController = a_motor.getPIDController();
        a_absEncoder = a_motor.getAbsoluteEncoder();
        a_pidController.setFeedbackDevice(a_absEncoder);

        a_kP = 4;
        a_kI = 0.3;
        a_kD = 0.06;
        a_kIz = 0.005;
        a_kFF = 0.0003;
        a_kMaxOutput = 10;
        a_kMinOutput = -1;
        a_maxRPM = 5700;

        // set PID coefficients
        a_pidController.setP(a_kP);
        a_pidController.setI(a_kI);
        a_pidController.setD(a_kD);
        a_pidController.setIZone(a_kIz);
        a_pidController.setFF(a_kFF);
        a_pidController.setOutputRange(a_kMinOutput, a_kMaxOutput);

        a_encoder = a_motor.getEncoder();
        a_pidController = a_motor.getPIDController();

        // a_encode(a_encoder.getPosition());

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", a_kP);
        SmartDashboard.putNumber("I Gain", a_kI);
        SmartDashboard.putNumber("D Gain", a_kD);
        SmartDashboard.putNumber("I Zone", a_kIz);
        SmartDashboard.putNumber("Feed Forward", a_kFF);
        SmartDashboard.putNumber("Max Output", a_kMaxOutput);
        SmartDashboard.putNumber("Min Output", a_kMinOutput);
        SmartDashboard.putNumber("Manual Position", 0);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("A_Encoder", a_encoder.getPosition());
        SmartDashboard.putNumber("Absolute encoder reading", a_absEncoder.getPosition());


        Logger.recordOutput("Assembly Motor Speed", a_motor.get());
        Logger.recordOutput("Assembly Encoder Position", a_encoder.getPosition());

        double p = SmartDashboard.getNumber("P Gain", 0.05);
    double i = SmartDashboard.getNumber("I Gain", 0.05);
    double d = SmartDashboard.getNumber("D Gain", 0.003);
    double iz = SmartDashboard.getNumber("I Zone", 0.001);
    double ff = SmartDashboard.getNumber("Feed Forward", 0.0015);
    double max = SmartDashboard.getNumber("Max Output", 10);
    double min = SmartDashboard.getNumber("Min Output", -1);
    // setRawPosition(SmartDashboard.getNumber("Manual Position", 0));
    


    if((p != a_kP)) { a_pidController.setP(p); a_kP = p; }
    if((i != a_kI)) { a_pidController.setI(i); a_kI = i; }
    if((d != a_kD)) { a_pidController.setD(d); a_kD = d; }
    if((iz != a_kIz)) { a_pidController.setIZone(iz); a_kIz = iz; }
    if((ff != a_kFF)) { a_pidController.setFF(ff); a_kFF = ff; }
    if((max != a_kMaxOutput) || (min != a_kMinOutput)) { 
      a_pidController.setOutputRange(min, max); 
      a_kMinOutput = min; a_kMaxOutput = max; 
    }
    }

    public void setRawPosition(double position) {
        a_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeed(double speed) {
        a_motor.set(speed * AssemblyConstants.ASSEMBLY_MAX_SPEED);
    }

    public void shootingPosition() {

        a_pidController.setReference(AssemblyConstants.ASSEMBLY_SHOOTING_ANGLE, CANSparkMax.ControlType.kPosition);
    }

    public void newshootingPosition() {
        a_pidController.setP(4);
        a_pidController.setReference(AssemblyConstants.ASSEMBLY_NEW_SHOOTING_ANGLE, CANSparkMax.ControlType.kPosition);
    }

    public void intakePosition() {
        a_pidController.setP(2);
        a_pidController.setReference(AssemblyConstants.ASSEMBLY_INTAKE_ANGLE, CANSparkMax.ControlType.kPosition);
    }

    public void autonomousPosition() {
        a_pidController.setReference(AssemblyConstants.ASSEMBLY_AUTONOMOUS1_SHOOTING_ANGLE,
                CANSparkMax.ControlType.kPosition);
    }

    public void passPosition() {
        a_pidController.setP(4);
        a_pidController.setReference(AssemblyConstants.ASSEMBLY_PASS_ANGLE, CANSparkMax.ControlType.kPosition);
    }

    public void ampShoot() {
        a_pidController.setReference(AssemblyConstants.ASSEMBLY_AMP_SHOOT, CANSparkMax.ControlType.kPosition);
    }

    public void resetEncoder() {
        a_encoder.setPosition(0);
    }
}