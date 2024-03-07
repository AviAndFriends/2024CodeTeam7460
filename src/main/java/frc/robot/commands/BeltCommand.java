package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BeltConstants;
import frc.robot.subsystems.BeltSubsystem;

public class BeltCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BeltSubsystem beltSubsystem;

    public BeltCommand(BeltSubsystem bsubsystem) {
        beltSubsystem = bsubsystem;
        addRequirements(beltSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
        beltSubsystem.setSpeed(BeltConstants.BELT_MAX_SPEED);
    }


        @Override
        public void end(boolean interrupted){
            beltSubsystem.setSpeed(0);
        }

        
@Override
public boolean isFinished() {
    return false;
}
}
