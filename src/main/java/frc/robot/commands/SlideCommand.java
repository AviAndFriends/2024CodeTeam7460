package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SlideConstants;
import frc.robot.subsystems.SlideSubsystem;


public class SlideCommand extends Command{

<<<<<<< HEAD
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlideSubsystem slideSubsystem;

    public SlideCommand(SlideSubsystem ssubsystem) {
        slideSubsystem = ssubsystem;
        addRequirements(slideSubsystem);
=======
    private final SlideSubsystem slideSS;
    private final double speed;

    public SlideCommand(SlideSubsystem slideSS, double speed) {
        this.slideSS = slideSS;
        this.speed = speed;
        addRequirements(slideSS);
>>>>>>> 871ffe4fac66ff29b1b72c08cfffe7cde5e453d6
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
        slideSubsystem.setSpeed(SlideConstants.SLIDE_MAX_SPEED);
        
    }

<<<<<<< HEAD

        @Override
        public void end(boolean interrupted){
            slideSubsystem.setSpeed(0);
        }
=======
    @Override
    public void execute() {
        slideSS.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        slideSS.setSpeed(0);
    }
>>>>>>> 871ffe4fac66ff29b1b72c08cfffe7cde5e453d6

        
@Override
public boolean isFinished() {
    return false;
}
}
