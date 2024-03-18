package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.AssemblySubsystem;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;

public class ShootCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BeltSubsystem beltSubsystem;
    private final FlyWheelSubsystem flyWheelSubsystem;
    private final AssemblySubsystem assemblySubsystem;

    public ShootCommand(BeltSubsystem bsubsystem, FlyWheelSubsystem fsubsystem, AssemblySubsystem asubsystem) {
        beltSubsystem = bsubsystem;
        flyWheelSubsystem = fsubsystem;
        assemblySubsystem = asubsystem;

addRequirements(beltSubsystem, flyWheelSubsystem, assemblySubsystem);

    }

   @Override 
    public void initialize() {
        assemblySubsystem.shootingPosition();
        flyWheelSubsystem.setSpeed (FlyWheelConstants.FW_MAX_SPEED);
        beltSubsystem.setSpeed(BeltConstants.BELT_MAX_SPEED); 
    
    }

    @Override
    public void execute(){
        flyWheelSubsystem.setSpeed (FlyWheelConstants.FW_MAX_SPEED);
        beltSubsystem.setSpeed(BeltConstants.BELT_MAX_SPEED); 
         
    }


    @Override
    public void end(boolean interrupted){
            beltSubsystem.setSpeed(0);
            flyWheelSubsystem.setSpeed(0);

     
    }

        
@Override
public boolean isFinished() {
    return false;
}
}

    

