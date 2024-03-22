package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlyWheelConstants;

public class FlyWheelSubsystem extends SubsystemBase { 

    private final CANSparkMax fw_motor = new CANSparkMax(FlyWheelConstants.FW_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    
    public FlyWheelSubsystem() {
        fw_motor.restoreFactoryDefaults();
    }

    @Override
    public void periodic() {
        double rpm = fw_motor.getEncoder().getVelocity();
        boolean ready = Math.abs(rpm) > 3200;
        Logger.recordOutput("Flywheel Motor Speed", fw_motor.get());
        SmartDashboard.putNumber("FlyWheel RPM", rpm);
        SmartDashboard.putBoolean("Shoot Ready", ready);
    }

    public void setSpeed(double speed) {
        fw_motor.set(speed);
    }
}
