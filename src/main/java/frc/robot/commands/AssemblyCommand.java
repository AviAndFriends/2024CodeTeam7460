package frc.robot.commands;

import java.security.PublicKey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AssemblySubsystem;


public class AssemblyCommand extends Command {
    
    private final AssemblySubsystem assemblySS;
    private final double speed;

    public AssemblyCommand(AssemblySubsystem assemblySS, double speed) {
        this.assemblySS = assemblySS;
        this.speed = speed;
        addRequirements(assemblySS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        assemblySS.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        assemblySS.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
