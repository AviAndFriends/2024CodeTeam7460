package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SlideConstants;
import frc.robot.subsystems.SlideSubsystem;


public class DownSlideCommand extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlideSubsystem slideSubsystem;

    public DownSlideCommand(SlideSubsystem ssubsystem) {
        slideSubsystem = ssubsystem;
        addRequirements(slideSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void execute(){
        
        slideSubsystem.setSpeed(-SlideConstants.SLIDE_MAX_SPEED);
    }


        @Override
        public void end(boolean interrupted){
            slideSubsystem.setSpeed(0);
            slideSubsystem.lowPosition();
        }

        
@Override
public boolean isFinished() {
    return false;
}
}
