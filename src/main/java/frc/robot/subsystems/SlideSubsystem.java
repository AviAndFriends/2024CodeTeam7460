package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlideConstants;

public class SlideSubsystem extends SubsystemBase{
    private final CANSparkMax s_motor = new CANSparkMax(SlideConstants.SLIDE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    private AbsoluteEncoder s_encoder;
    

    public SlideSubsystem() {
        s_motor.restoreFactoryDefaults();
    }

    @Override
    public void periodic() {
        s_encoder = s_motor.getAbsoluteEncoder();
        SmartDashboard.putNumber("Encoder", s_encoder.getPosition());

        Logger.recordOutput("Slide Motor Speed", s_motor.get());
        Logger.recordOutput("Slide Encoder Position", s_encoder.getPosition());
    }

    public void setSpeed(double speed) {
        s_motor.set(speed * SlideConstants.SLIDE_MAX_SPEED);
    }
}
