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

public class ResetAssemblyEncoderCommand extends Command {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final AssemblySubsystem assemblySubsystem;
    private boolean done;

    public ResetAssemblyEncoderCommand(AssemblySubsystem assemblySubsystem) {
        this.assemblySubsystem = assemblySubsystem;
        addRequirements(assemblySubsystem);
        done = false;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        assemblySubsystem.resetEncoder();
        done = true;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
