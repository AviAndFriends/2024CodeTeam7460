package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SlideConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


public class SlideSubsystem extends SubsystemBase{
    private final CANSparkMax s_motor = new CANSparkMax(SlideConstants.SLIDE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    
    private RelativeEncoder s_encoder;
    private SparkPIDController s_pidController;
    public double s_kP, s_kI, s_kD, s_kIz, s_kFF, s_kMaxOutput, s_kMinOutput, s_maxRPM;


    public SlideSubsystem() {

        s_motor.restoreFactoryDefaults();
        s_pidController = s_motor.getPIDController();
    
        s_kP = 0.05;
        s_kI = 0.05;
        s_kD = 0.003;
        s_kIz = 0.001;
        s_kFF = 0.0015;
        s_kMaxOutput = 10;
        s_kMinOutput = -1;
        s_maxRPM = 5700;
  
        // set PID coefficients
        s_pidController.setP(s_kP);
        s_pidController.setI(s_kI);
        s_pidController.setD(s_kD);
        s_pidController.setIZone(s_kIz);
        s_pidController.setFF(s_kFF);
        s_pidController.setOutputRange(s_kMinOutput, s_kMaxOutput);

        s_encoder = s_motor.getEncoder();
        s_pidController = s_motor.getPIDController();
    
        // s_encoder(s_encoder.getPosition());
    }


    @Override
    public void periodic() {
    

        SmartDashboard.putNumber("S_Encoder", s_encoder.getPosition());

        Logger.recordOutput("Slide Motor Speed", s_motor.get());
       Logger.recordOutput("Slide Encoder Position", s_encoder.getPosition());
    }

    public void setSpeed(double speed) {
        s_motor.set(speed * SlideConstants.SLIDE_MAX_SPEED);
    }

    public void lowPosition(){
    
        s_pidController.setReference(SlideConstants.SLIDE_LOW ,CANSparkMax.ControlType.kPosition);
    }   

    public void highPosition(){
        s_pidController.setReference(SlideConstants.SLIDE_HIGH ,CANSparkMax.ControlType.kPosition);

    }

    public void slideAmpShoot(){
        s_pidController.setReference(SlideConstants.SLIDE_AMP_SHOOT ,CANSparkMax.ControlType.kPosition);

    }

    // public void slideLow(){
    //     s_pidController.setReference(SlideConstants.SLIDE_LOW, CANSparkMax.ControlType.kPosition);     
    // }

}
