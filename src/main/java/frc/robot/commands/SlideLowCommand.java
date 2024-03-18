package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.subsystems.AssemblySubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.Constants.SlideConstants;

public class SlideLowCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlideSubsystem slideSubsystem;
    private final AssemblySubsystem assemblySubsystem;

    public SlideLowCommand(SlideSubsystem sSubsystem, AssemblySubsystem aSubsystem) {
        slideSubsystem = sSubsystem;
        assemblySubsystem = aSubsystem;
        addRequirements(slideSubsystem, assemblySubsystem);
}

@Override 
    public void initialize() {}

    @Override
    public void execute(){

     assemblySubsystem.intakePosition();
    slideSubsystem.lowPosition();
   

}

}