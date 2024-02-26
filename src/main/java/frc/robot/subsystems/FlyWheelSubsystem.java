package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlyWheelConstants;

public class FlyWheelSubsystem extends SubsystemBase { 

    private final CANSparkMax fw_motor = new CANSparkMax(FlyWheelConstants.FW_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    
    public FlyWheelSubsystem() {

    }

    @Override
    public void periodic() {
        Logger.recordOutput("Flywheel Motor Speed", fw_motor.get());
    }

    public void setSpeed(double speed) {
        fw_motor.set(speed);
    }
}
