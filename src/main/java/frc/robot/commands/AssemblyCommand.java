package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.subsystems.AssemblySubsystem;


public class AssemblyCommand extends Command {
<<<<<<< HEAD
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AssemblySubsystem assemblySubsystem;

    public AssemblyCommand(AssemblySubsystem asubsystem) {
        assemblySubsystem = asubsystem;
        addRequirements(assemblySubsystem);
=======
    
    private final AssemblySubsystem assemblySS;
    private final double speed;

    public AssemblyCommand(AssemblySubsystem assemblySS, double speed) {
        this.assemblySS = assemblySS;
        this.speed = speed;
        addRequirements(assemblySS);
>>>>>>> 871ffe4fac66ff29b1b72c08cfffe7cde5e453d6
    }

    @Override 
    public void initialize() {}

    @Override
<<<<<<< HEAD
    public void execute(){
        assemblySubsystem.setSpeed(AssemblyConstants.ASSEMBLY_MAX_SPEED);
    }

=======
    public void execute() {
        assemblySS.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        assemblySS.setSpeed(0);
    }
>>>>>>> 871ffe4fac66ff29b1b72c08cfffe7cde5e453d6

        @Override
        public void end(boolean interrupted){
            assemblySubsystem.setSpeed(0);
        }

        
@Override
public boolean isFinished() {
    return false;
}
}
