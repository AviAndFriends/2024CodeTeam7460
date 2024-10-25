package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.BeltConstants;
import frc.robot.subsystems.BeltSubsystem;



public class DownBeltCommand extends Command{

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BeltSubsystem beltSubsystem;

    public DownBeltCommand(BeltSubsystem bsubsystem) {
        beltSubsystem = bsubsystem;
        addRequirements(beltSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
        beltSubsystem.setSpeed(-0.5);
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
