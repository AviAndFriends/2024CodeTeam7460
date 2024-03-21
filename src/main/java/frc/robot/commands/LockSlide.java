package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SlideSubsystem;

public class LockSlide extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final SlideSubsystem slideSubsystem;

    public LockSlide(SlideSubsystem ssubsystem) {
        slideSubsystem = ssubsystem;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        slideSubsystem.lowPosition();

    }
}
