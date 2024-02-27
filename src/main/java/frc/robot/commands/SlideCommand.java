package frc.robot.commands;

import java.security.PublicKey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SlideSubsystem;


public class SlideCommand extends Command{

    private final SlideSubsystem slideSS;
    private final double speed;

    public SlideCommand(SlideSubsystem slideSS, double speed) {
        this.slideSS = slideSS;
        this.speed = speed;
        addRequirements(slideSS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        slideSS.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        slideSS.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }    
    
}
