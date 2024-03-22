package frc.robot.subsystems;

import javax.script.AbstractScriptEngine;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AssemblyConstants;


public class AssemblySubsystem extends SubsystemBase { 
    private final CANSparkMax a_motor = new CANSparkMax(AssemblyConstants.ASSEMBLY_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    private RelativeEncoder a_encoder;
    private SparkPIDController a_pidController;
    public double a_kP, a_kI, a_kD, a_kIz, a_kFF, a_kMaxOutput, a_kMinOutput, a_maxRPM;
    

    public AssemblySubsystem() {
    
        a_motor.restoreFactoryDefaults();
        a_pidController = a_motor.getPIDController();

        a_kP = 0.05;
        a_kI = 0.05;
        a_kD = 0.003;
        a_kIz = 0.001;
        a_kFF = 0.0015;
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
        
        //a_encode(a_encoder.getPosition());
    }

    @Override
    public void periodic() {
        

        SmartDashboard.putNumber("A_Encoder", a_encoder.getPosition());

        Logger.recordOutput("Assembly Motor Speed", a_motor.get());
        Logger.recordOutput("Assembly Encoder Position", a_encoder.getPosition());
    }

    public void setSpeed(double speed) {
        a_motor.set(speed * AssemblyConstants.ASSEMBLY_MAX_SPEED);
    }

    public void shootingPosition() {
        
        a_pidController.setReference(AssemblyConstants.ASSEMBLY_SHOOTING_ANGLE,CANSparkMax.ControlType.kPosition); 
    }

    public void intakePosition() {
        a_pidController.setReference(AssemblyConstants.ASSEMBLY_INTAKE_ANGLE, CANSparkMax.ControlType.kPosition); 
    }

    public void autonomousPosition() {
        a_pidController.setReference(AssemblyConstants.ASSEMBLY_AUTONOMOUS1_SHOOTING_ANGLE, CANSparkMax.ControlType.kPosition);
    }

    public void ampShoot(){
        a_pidController.setReference(AssemblyConstants.ASSEMBLY_AMP_SHOOT, CANSparkMax.ControlType.kPosition);     
    }
}