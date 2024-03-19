// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AssemblyCommand;
import frc.robot.commands.AssemblyIntakeCommand;
import frc.robot.commands.AssemblyShootingCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.Autonomous1ReadyToShoot;
import frc.robot.commands.BeltCommand;
import frc.robot.commands.DownAssemblyCommand;
import frc.robot.commands.DownBeltCommand;
import frc.robot.commands.DownSlideCommand;
import frc.robot.commands.FlyWheelCommand;
import frc.robot.commands.GroundLoadCommand;
import frc.robot.commands.IntakeCom3;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.AmpReadyCommand;
import frc.robot.commands.ReadyToShootCommand;
import frc.robot.commands.RealAutonomousCommand;
import frc.robot.commands.ReverseBeltCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SlideCommand;
import frc.robot.commands.SlideHighCommand;
import frc.robot.commands.SlideLowCommand;
import frc.robot.subsystems.AssemblySubsystem;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import java.util.List;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

// import org.photonvision.simulation.VisionSystemSim;
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final BeltSubsystem beltSubsystem = new BeltSubsystem();
  private final FlyWheelSubsystem flyWheelSubsystem = new FlyWheelSubsystem();
  private final AssemblySubsystem assemblySubsystem = new AssemblySubsystem();
  private final SlideSubsystem slideSubsystem = new SlideSubsystem();
  private final UltrasonicSubsystem ultrasonicSubsystem = new UltrasonicSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final RealAutonomousCommand realAutonomousCommand = new RealAutonomousCommand(beltSubsystem, flyWheelSubsystem, assemblySubsystem);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController supplementalController = new XboxController(OIConstants.SUPPLEMENTAL_CONTROLLER_PORT);
  XboxController buttonBox = new XboxController(OIConstants.BUTTON_BOX_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

   m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive)); 
  

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));


    // new JoystickButton(supplementalController, XboxController.Button.kY.value)
    // .toggleOnTrue(new GroundLoadCommand(beltSubsystem, intakeSubsystem, assemblySubsystem, flyWheelSubsystem));
    
    
    new JoystickButton(supplementalController, XboxController.Button.kY.value)
    .onTrue(new AlignCommand(m_robotDrive, visionSubsystem));
    

    new JoystickButton(supplementalController, XboxController.Button.kB.value)
    .toggleOnTrue(new ReadyToShootCommand(flyWheelSubsystem, assemblySubsystem));

    
    //new JoystickButton(supplementalController,XboxController.Button.kX.value)
    //.whileTrue(new ShootCommand(beltSubsystem, flyWheelSubsystem, assemblySubsystem));
    
    new JoystickButton(m_driverController,XboxController.Button.kA.value)
    .whileTrue (new BeltCommand (beltSubsystem));


    new JoystickButton(m_driverController, XboxController.Button.kB.value)
    .whileTrue(new DownAssemblyCommand(assemblySubsystem));

    //new JoystickButton(supplementalController,XboxController.Button.kX.value)
    //.whileTrue(new AssemblyShootingCommand(assemblySubsystem));

    //new JoystickButton(supplementalController, XboxController.Button.kY.value)
    //.whileTrue(new AssemblyIntakeCommand(assemblySubsystem));


    new POVButton(supplementalController, 0)
    .whileTrue(new AssemblyCommand(assemblySubsystem));

    new JoystickButton(supplementalController, XboxController.Button.kRightBumper.value)
    .whileTrue(new GroundLoadCommand(beltSubsystem, intakeSubsystem, assemblySubsystem, flyWheelSubsystem));
   
    new JoystickButton(supplementalController, XboxController.Button.kLeftBumper.value)
    .whileTrue(new ReverseBeltCommand(beltSubsystem));

   
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    .whileTrue(new SlideCommand(slideSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    .whileTrue(new DownSlideCommand(slideSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
    .toggleOnTrue(new AmpReadyCommand(slideSubsystem, assemblySubsystem));

    
    new JoystickButton(buttonBox,1)
    .whileTrue(new AssemblyCommand(assemblySubsystem));
   
    new JoystickButton(buttonBox,2)
    .whileTrue(new DownAssemblyCommand(assemblySubsystem));


    new JoystickButton(buttonBox,3)
    .toggleOnTrue(new AssemblyShootingCommand(assemblySubsystem));


    new JoystickButton(buttonBox,4)
    .toggleOnTrue(new AssemblyIntakeCommand(assemblySubsystem));
   


   new JoystickButton(buttonBox, 5)
   .whileTrue(new SlideCommand(slideSubsystem));


   new JoystickButton(buttonBox, 6)
   .whileTrue(new DownSlideCommand(slideSubsystem));


   new JoystickButton(buttonBox, 7)
   .toggleOnTrue(new SlideLowCommand(slideSubsystem, assemblySubsystem));


   new JoystickButton(buttonBox, 8)
   .toggleOnTrue(new SlideHighCommand(slideSubsystem));


  new JoystickButton(buttonBox, 9)
  .whileTrue(new BeltCommand(beltSubsystem));


  new JoystickButton(buttonBox, 10)
  .whileTrue(new DownBeltCommand(beltSubsystem));


  new JoystickButton(buttonBox, 11)
  .toggleOnTrue(new FlyWheelCommand(flyWheelSubsystem));


  new JoystickButton(buttonBox, 12)
  .toggleOnTrue(new IntakeCom3(intakeSubsystem));

  

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
       return new SequentialCommandGroup(
        //new AutoDriveCommand(),
        //new WaitCommand(5),
        //new RealAutonomousCommand(beltSubsystem, flyWheelSubsystem, assemblySubsystem)
      );

      
          //new ParallelDeadlineGroup(
            //new WaitCommand(3), 
            //new RunCommand(m_robotDrive::setX)
        //),
        //new ShootCommand(beltSubsystem, flyWheelSubsystem, assemblySubsystem))

  }
}