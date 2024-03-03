package frc.robot.commands;

import java.security.PublicKey;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    
    private final IntakeSubsystem intakeSS;
    private final double speed;

    public IntakeCommand(IntakeSubsystem intakeSS, double speed) {
        this.intakeSS = intakeSS;
        this.speed = speed;
<<<<<<< HEAD
        addRequirements(this.intakeSubsystem);
=======
        addRequirements(intakeSS);
>>>>>>> 871ffe4fac66ff29b1b72c08cfffe7cde5e453d6
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSS.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSS.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
