package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.IntakeConstants;

public class BeltSubsystem extends SubsystemBase { 
    
    private final CANSparkMax b_motor = new CANSparkMax(BeltConstants.BELT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    
    double b_backup = BeltConstants.BELT_BACKUP_OFFEST;
    double initTime;
    double speed = BeltConstants.BELT_MAX_SPEED;
    
    public BeltSubsystem() {
        b_motor.restoreFactoryDefaults();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Belt Motor Speed", b_motor.get());
    }

    public void setSpeed(double speed) {
        b_motor.set(speed);
    }

    public void backup() {

    initTime = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp()- initTime <= b_backup){
        // System.out.println(Timer.getFPGATimestamp()-initTime);
        b_motor.set(-speed);
    }
    b_motor.set(0);


    //     b_backup= b_encoder.getPosition() - BeltConstants.BELT_BACKUP_OFFEST;
    //     b_pidController.setReference(b_backup, CANSparkMax.ControlType.kPosition);
    }

    public void stop(){
        b_motor.stopMotor();
    }
}
