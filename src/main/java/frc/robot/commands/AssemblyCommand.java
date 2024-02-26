package frc.robot.commands;

import java.security.PublicKey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AssemblySubsystem;


public class AssemblyCommand extends Command {
    
    private final AssemblySubsystem assemblySubsystem;
    private final double speed;

    public AssemblyCommand(AssemblySubsystem assemblySubsystem, double speed) {
        this.assemblySubsystem = assemblySubsystem;
        this.speed = speed;
        addRequirements(assemblySubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        assemblySubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        assemblySubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
