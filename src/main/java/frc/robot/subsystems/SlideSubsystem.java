package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SlideConstants;

public class SlideSubsystem extends SubsystemBase{
    private final CANSparkMax s_motor = new CANSparkMax(SlideConstants.SLIDE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder s_encoder;
    

    public SlideSubsystem() {
        
    }

    @Override
    public void periodic() {
        s_encoder = s_motor.getEncoder();

        Logger.recordOutput("Assembly Motor Speed", s_motor.get());
        Logger.recordOutput("Assembly Encoder Position", s_encoder.getPosition());
    }

    public void setSpeed(double speed) {
        s_motor.set(speed * SlideConstants.SLIDE_MAX_SPEED);
    }
    
}
