package frc.robot.commands;

<<<<<<< HEAD
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
=======
import java.security.PublicKey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeltSubsystem;


public class BeltCommand extends Command {
    
    private final BeltSubsystem beltSS;
    private final double speed;

    public BeltCommand(BeltSubsystem beltSS, double speed) {
        this.beltSS = beltSS;
        this.speed = speed;
        addRequirements(beltSS);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        beltSS.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        beltSS.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
>>>>>>> 871ffe4fac66ff29b1b72c08cfffe7cde5e453d6
}
