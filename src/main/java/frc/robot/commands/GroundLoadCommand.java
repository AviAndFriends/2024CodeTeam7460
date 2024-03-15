package frc.robot.commands;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.AssemblySubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;


public class GroundLoadCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BeltSubsystem beltSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    //private final UltrasonicSubsystem ultrasonicSubsystem;
    private final AssemblySubsystem assemblySubsystem;
    private final FlyWheelSubsystem flyWheelSubsystem;

    public GroundLoadCommand(BeltSubsystem bsubsystem, IntakeSubsystem isubsystem,
        AssemblySubsystem asubsystem, FlyWheelSubsystem fsubsystem) {
        beltSubsystem = bsubsystem;
        intakeSubsystem = isubsystem;
        //ultrasonicSubsystem = usubsystem;
        assemblySubsystem = asubsystem;
        flyWheelSubsystem = fsubsystem;

        addRequirements(beltSubsystem, intakeSubsystem, assemblySubsystem, flyWheelSubsystem);
    }



    @Override 
    public void initialize() {
        assemblySubsystem.intakePosition();
        flyWheelSubsystem.setSpeed (0);
      
             
    }

    @Override
    public void execute(){
        beltSubsystem.setSpeed(BeltConstants.BELT_MAX_SPEED);
        intakeSubsystem.setSpeed(IntakeConstants.INTAKE_MAX_SPEED);
        //if(ultrasonicSubsystem.loaded) {
          //  end (false);
        
      //  }
            


           // end(false);
        }



    @Override
    public void end(boolean interrupted){

            intakeSubsystem.setSpeed(0);
           beltSubsystem.setSpeed(0);
           // assemblySubsystem.shootingPosition();
           // flyWheelSubsystem.setSpeed(FlyWheelConstants.FW_MAX_SPEED);

     
    }

        
@Override
public boolean isFinished() {
   


    return false;
}
}
