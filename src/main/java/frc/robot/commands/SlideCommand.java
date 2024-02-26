package frc.robot.commands;

import java.security.PublicKey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SlideSubsystem;


public class SlideCommand extends Command{

    private final SlideSubsystem slideSubsystem;
    private final double speed;

    public SlideCommand(SlideSubsystem slideSubsystem, double speed) {
        this.slideSubsystem = slideSubsystem;
        this.speed = speed;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        slideSubsystem.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        slideSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }    
    
}
