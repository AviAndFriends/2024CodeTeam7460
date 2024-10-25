package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.AssemblySubsystem;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class RealAutonomousCommand extends Command {
    
     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BeltSubsystem beltSubsystem;
    private final FlyWheelSubsystem flyWheelSubsystem;
    private final AssemblySubsystem assemblySubsystem;

    public RealAutonomousCommand(BeltSubsystem bsubsystem, FlyWheelSubsystem fsubsystem, AssemblySubsystem asubsystem) {
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
