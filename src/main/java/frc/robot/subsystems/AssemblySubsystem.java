package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.Constants.IntakeConstants;

public class AssemblySubsystem extends SubsystemBase { 

    private final CANSparkMax a_motor = new CANSparkMax(AssemblyConstants.ASSEMBLY_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder a_encoder;
    

    public AssemblySubsystem() {
        
    }

    @Override
    public void periodic() {
        a_encoder = a_motor.getEncoder();

        Logger.recordOutput("Assembly Motor Speed", a_motor.get());
        Logger.recordOutput("Assembly Encoder Position", a_encoder.getPosition());
    }

    public void setSpeed(double speed) {
        a_motor.set(speed * AssemblyConstants.ASSEMBLY_MAX_SPEED);
    }
}
