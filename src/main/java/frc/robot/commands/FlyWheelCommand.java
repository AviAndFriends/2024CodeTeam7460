package frc.robot.commands;

import java.security.PublicKey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;


public class FlyWheelCommand extends Command {
    
    private final FlyWheelSubsystem flywheelSS;
    private final double speed;

    public FlyWheelCommand(FlyWheelSubsystem flywheelSS, double speed) {
        this.flywheelSS = flywheelSS;
        this.speed = speed;
        addRequirements(flywheelSS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        flywheelSS.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSS.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
