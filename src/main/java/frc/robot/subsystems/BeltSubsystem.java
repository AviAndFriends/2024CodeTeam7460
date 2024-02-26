package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.IntakeConstants;

public class BeltSubsystem extends SubsystemBase { 

    private final CANSparkMax b_motor = new CANSparkMax(BeltConstants.BELT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    
    public BeltSubsystem() {
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Belt Motor Speed", b_motor.get());
    }

    public void setSpeed(double speed) {
        b_motor.set(speed);
    }
}
