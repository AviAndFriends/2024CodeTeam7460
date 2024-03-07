package frc.robot.commands;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.Constants.BeltConstants;
import frc.robot.subsystems.BeltSubsystem;


public class GroundLoadCommand extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BeltSubsystem beltSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final UltrasonicSubsystem ultrasonicSubsystem;

    public GroundLoadCommand(BeltSubsystem bsubsystem, IntakeSubsystem isubsystem, UltrasonicSubsystem usubsystem) {
        beltSubsystem = bsubsystem;
        intakeSubsystem = isubsystem;
        ultrasonicSubsystem = usubsystem;
        addRequirements(beltSubsystem, intakeSubsystem, ultrasonicSubsystem);
    }



    @Override 
    public void initialize() {}

    @Override
    public void execute(){
            beltSubsystem.setSpeed(BeltConstants.BELT_MAX_SPEED);
            intakeSubsystem.setSpeed(IntakeConstants.INTAKE_MAX_SPEED); 
        if(ultrasonicSubsystem.loaded) {
            end(false);
        }
    }


    @Override
    public void end(boolean interrupted){
            beltSubsystem.setSpeed(0);
            intakeSubsystem.setSpeed(0);
     
    }

        
@Override
public boolean isFinished() {
    //ultrasonicSubsystem = loaded;
    return false;
}
}
