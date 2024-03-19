package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private boolean done;
    

    public AlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        // if(visionSubsystem.getXPosition(7) < 1) {
        // driveSubsystem.drive(-0.3, 0, 0, false, false);
        // } else if(visionSubsystem.getXPosition(7) > 1.1) {
        // driveSubsystem.drive(0.3, 0, 0, false, false);
        // } else {
        // driveSubsystem.drive(0, 0, 0, false, false);
        // }
     double yPos;
        if(visionSubsystem.getYPosition(7) != 10000) {
            yPos = visionSubsystem.getYPosition(7);
        } else if(visionSubsystem.getYPosition(4) != 10000) {
            yPos = visionSubsystem.getYPosition(4);
        } else {
            yPos = 10000;
        }
        

        if (yPos > -0.12 && yPos != 10000) {
            driveSubsystem.drive(0, 0, 0.2, false, false);
        } else if (yPos < -0.28 && yPos != 10000) {
            driveSubsystem.drive(0, 0, -0.2, false, false);
        } else {
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
