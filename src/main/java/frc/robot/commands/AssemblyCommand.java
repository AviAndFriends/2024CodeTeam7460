package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.subsystems.AssemblySubsystem;


public class AssemblyCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AssemblySubsystem assemblySubsystem;

    public AssemblyCommand(AssemblySubsystem asubsystem) {
        assemblySubsystem = asubsystem;
        addRequirements(assemblySubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
        assemblySubsystem.setSpeed(AssemblyConstants.ASSEMBLY_MAX_SPEED);
    }


        @Override
        public void end(boolean interrupted){
            assemblySubsystem.setSpeed(0);
        }

        
@Override
public boolean isFinished() {
    return false;
}
}
