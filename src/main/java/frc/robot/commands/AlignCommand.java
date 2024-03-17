package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AssemblySubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.AssemblyConstants;
import frc.robot.Constants.SlideConstants;

public class AlignCommand extends Command {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    boolean finished;


    public AlignCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        finished = false;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // if(visionSubsystem.getXPosition(7) < 1) {
        //     driveSubsystem.drive(-0.3, 0, 0, false, false);
        // } else if(visionSubsystem.getXPosition(7) > 1.1) {
        //     driveSubsystem.drive(0.3, 0, 0, false, false);
        // } else {
        //     driveSubsystem.drive(0, 0, 0, false, false);
        // }
        double yPos = visionSubsystem.getYPosition(7);
        if(yPos > 0.05 && yPos != 10000) {
            driveSubsystem.drive(0, 0, 0.1, false, false);
        } else if(yPos < -0.05 && yPos != 10000) {
            driveSubsystem.drive(0, 0, -0.1, false, false);
        } else  {
            finished = true;
        }

        
        
    } 

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
