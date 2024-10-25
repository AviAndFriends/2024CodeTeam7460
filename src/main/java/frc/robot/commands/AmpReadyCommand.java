package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AssemblySubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.Constants.SlideConstants;


public class AmpReadyCommand  extends Command{
   
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlideSubsystem slideSubsystem;
    private final AssemblySubsystem assemblySubsystem;


    public AmpReadyCommand(SlideSubsystem ssubsystem, AssemblySubsystem asubsystem) {
        slideSubsystem = ssubsystem;
        assemblySubsystem = asubsystem;
        addRequirements(slideSubsystem, assemblySubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute(){    
        slideSubsystem.highPosition(); 
        assemblySubsystem.ampShoot();
            
        
        
    }

    @Override
    public void end(boolean interrupted){
            // slideSubsystem.lowPosition();     
    }
  
@Override
public boolean isFinished() {
    return false;
}
}


 