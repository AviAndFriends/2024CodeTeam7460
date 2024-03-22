package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveUntilDistanceCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem driveSubsystem;
    private final double DISTANCE;
    private boolean done;
    private final double INITIAL_XPOSITION;
    private final boolean forward;
    private final double SPEED;

    public DriveUntilDistanceCommand(DriveSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.DISTANCE = distance;
        INITIAL_XPOSITION = driveSubsystem.getPose().getX();

        forward = true;
        this.SPEED = 0.5;

        addRequirements(this.driveSubsystem);
    }

    public DriveUntilDistanceCommand(DriveSubsystem driveSubsystem, double distance, boolean forward, boolean slow) {
        this.driveSubsystem = driveSubsystem;
        this.DISTANCE = distance;
        INITIAL_XPOSITION = driveSubsystem.getPose().getX();

        this.forward = forward;
        this.SPEED = 0.2;


        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        done = false;

    }

    @Override
    public void execute() {
        if (driveSubsystem.getPose().getX() < DISTANCE + INITIAL_XPOSITION && forward) {
            driveSubsystem.drive(SPEED, 0, 0, false, false);
        } else if(driveSubsystem.getPose().getX() < DISTANCE + INITIAL_XPOSITION && forward == false) {
            driveSubsystem.drive(SPEED * -1, 0, 0, false, false);
        }
        else {
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
