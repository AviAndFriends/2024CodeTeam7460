package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlideConstants;
import frc.robot.Constants;

public class SlideSubsystem extends SubsystemBase{
    private final CANSparkMax s_motor = new CANSparkMax(SlideConstants.SLIDE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    private SparkRelativeEncoder s_encoder;

    private SparkPIDController s_pidController;
    

    public SlideSubsystem() {
        s_motor.restoreFactoryDefaults();
        s_encoder.setPositionConversionFactor(0.1);
    
    }

    @Override
    public void periodic() {
        //s_encoder = s_motor.SparkRelativeEncoder();
        s_pidController = s_motor.getPIDController();

        SmartDashboard.putNumber("Encoder", s_encoder.getPosition());

        Logger.recordOutput("Slide Motor Speed", s_motor.get());
        Logger.recordOutput("Slide Encoder Position", s_encoder.getPosition());


    }

    public void setSpeed(double speed) {
        s_motor.set(speed * SlideConstants.SLIDE_MAX_SPEED);
    }

    /*public void resetOdometry(Pose2d pose) {
    resetEncoders();
    }

    private void resetEncoders() {
      s_encoder.setPosition(0);
    }

    */
}
