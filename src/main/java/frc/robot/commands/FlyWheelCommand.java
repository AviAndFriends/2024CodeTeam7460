package frc.robot.commands;

<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.subsystems.FlyWheelSubsystem;

public class FlyWheelCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final FlyWheelSubsystem flyWheelSubsystem;

    public FlyWheelCommand(FlyWheelSubsystem fwsubsystem) {
        flyWheelSubsystem = fwsubsystem;
        addRequirements(flyWheelSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
        flyWheelSubsystem.setSpeed(FlyWheelConstants.FW_MAX_SPEED);
    }

    @Override
        public void end(boolean interrupted){
            flyWheelSubsystem.setSpeed(0);
    }

        
@Override
public boolean isFinished() {
    return false;
}
=======
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
>>>>>>> 871ffe4fac66ff29b1b72c08cfffe7cde5e453d6
}
