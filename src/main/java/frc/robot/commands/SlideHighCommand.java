package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SlideConstants;
import frc.robot.subsystems.SlideSubsystem;

public class SlideHighCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlideSubsystem slideSubsystem;

    public SlideHighCommand(SlideSubsystem ssubsystem) {
        slideSubsystem = ssubsystem;
        addRequirements(slideSubsystem);
}

    @Override 
    public void initialize() {}

    @Override
    public void execute(){

    slideSubsystem.highPosition();

}
}
