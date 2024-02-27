package frc.robot.commands;

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
}
