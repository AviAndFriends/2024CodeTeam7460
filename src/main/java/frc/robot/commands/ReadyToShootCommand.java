package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.Constants.FlyWheelConstants;
import frc.robot.subsystems.AssemblySubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;

public class ReadyToShootCommand extends Command {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final FlyWheelSubsystem flyWheelSubsystem;
    private final AssemblySubsystem assemblySubsystem;

    boolean finished = false;

    public ReadyToShootCommand(FlyWheelSubsystem fwsubsystem, AssemblySubsystem asubsystem) {
        flyWheelSubsystem = fwsubsystem;
        assemblySubsystem = asubsystem;
        addRequirements(flyWheelSubsystem, assemblySubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        flyWheelSubsystem.setSpeed(FlyWheelConstants.FW_MAX_SPEED);
        assemblySubsystem.shootingPosition();
        // finished = true;
    }

    @Override
    public void end(boolean interrupted) {
        flyWheelSubsystem.setSpeed(0);
        assemblySubsystem.intakePosition();

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
