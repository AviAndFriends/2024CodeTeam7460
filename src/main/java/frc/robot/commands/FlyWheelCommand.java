package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.subsystems.FlyWheelSubsystem;

public class FlyWheelCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final FlyWheelSubsystem flyWheelSubsystem;

    public FlyWheelCommand(FlyWheelSubsystem fwsubsystem) {
        flyWheelSubsystem = fwsubsystem;
        addRequirements(flyWheelSubsystem);
    }

    

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
        flyWheelSubsystem.setSpeed(FlyWheelConstants.FW_MAX_SPEED);
    }

    @Override
        public void end(boolean interrupted){
            flyWheelSubsystem.setSpeed(0);
    }

        
@Override
public boolean isFinished() {
    return false;
}
}
