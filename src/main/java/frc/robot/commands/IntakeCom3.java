package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCom3 extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCom3(IntakeSubsystem isubsystem) {
        intakeSubsystem = isubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
        intakeSubsystem.setSpeed(IntakeConstants.INTAKE_MAX_SPEED);
    }


        @Override
        public void end(boolean interrupted){
            intakeSubsystem.setSpeed(0);
        }

        
@Override
public boolean isFinished() {
    return false;
}
}
