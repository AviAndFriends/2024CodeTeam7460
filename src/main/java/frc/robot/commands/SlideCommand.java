package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SlideConstants;
import frc.robot.subsystems.SlideSubsystem;

public class SlideCommand extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlideSubsystem slideSubsystem;

    public SlideCommand(SlideSubsystem ssubsystem) {
        slideSubsystem = ssubsystem;
        addRequirements(slideSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
        slideSubsystem.setSpeed(SlideConstants.SLIDE_MAX_SPEED);
        
    }


        @Override
        public void end(boolean interrupted){
            slideSubsystem.setSpeed(0);
        }

        
@Override
public boolean isFinished() {
    return false;
}
}
