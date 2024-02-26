package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase { 

    private final CANSparkMax i_motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    
    public IntakeSubsystem() {

    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake Motor Speed", i_motor.get());
    }

    public void setSpeed(double speed) {
        i_motor.set(speed);
    }
}
