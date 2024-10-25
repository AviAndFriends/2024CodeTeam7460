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

public class ReverseBeltCommand extends Command {
    
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BeltSubsystem beltSubsystem;

    public ReverseBeltCommand(BeltSubsystem bsubsystem) {
        beltSubsystem = bsubsystem;
        addRequirements(beltSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute(){
        beltSubsystem.setSpeed(BeltConstants.BELT_BACKUP_OFFEST);
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
