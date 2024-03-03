package frc.robot.subsystems;

import javax.script.AbstractScriptEngine;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AssemblyConstants;


public class AssemblySubsystem extends SubsystemBase { 

    private final CANSparkMax a_motor = new CANSparkMax(AssemblyConstants.ASSEMBLY_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    private AbsoluteEncoder a_encoder;
    

    public AssemblySubsystem() {
    
        a_motor.restoreFactoryDefaults();
    }


    @Override
    public void periodic() {
        a_encoder = a_motor.getAbsoluteEncoder();
        SmartDashboard.putNumber("Encoder", a_encoder.getPosition());

        Logger.recordOutput("Assembly Motor Speed", a_motor.get());
        Logger.recordOutput("Assembly Encoder Position", a_encoder.getPosition());
    }

    public void setSpeed(double speed) {
        a_motor.set(speed * AssemblyConstants.ASSEMBLY_MAX_SPEED);
    }
}