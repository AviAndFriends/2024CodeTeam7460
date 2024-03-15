package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.subsystems.AssemblySubsystem;

public class AssemblyIntakeCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AssemblySubsystem assemblySubsystem;

    public AssemblyIntakeCommand(AssemblySubsystem asubsystem) {
        assemblySubsystem = asubsystem;
        addRequirements(assemblySubsystem);
}

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
    
    assemblySubsystem.intakePosition();

}
}
