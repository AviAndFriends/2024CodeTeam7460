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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.commands.DriveUntilDistanceCommand;
import frc.robot.commands.FlyWheelCommand;
import frc.robot.commands.GroundLoadCommand;
import frc.robot.commands.IntakeCom3;
import frc.robot.commands.LockSlide;
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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {

        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final BeltSubsystem beltSubsystem = new BeltSubsystem();
        private final FlyWheelSubsystem flyWheelSubsystem = new FlyWheelSubsystem();
        private final AssemblySubsystem assemblySubsystem = new AssemblySubsystem();
        protected final SlideSubsystem slideSubsystem = new SlideSubsystem();
        private final UltrasonicSubsystem ultrasonicSubsystem = new UltrasonicSubsystem();
        private final VisionSubsystem visionSubsystem = new VisionSubsystem();

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

                PowerDistribution m_PowerDistribution = new PowerDistribution(20, ModuleType.kRev);

                SmartDashboard.putData(m_PowerDistribution);

                // push gyro
                SmartDashboard.putData(m_robotDrive.m_gyro);

                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true, true),
                                                m_robotDrive));

                NamedCommands.registerCommand("FlyWheel", new FlyWheelCommand(flyWheelSubsystem));
                NamedCommands.registerCommand("GroundLoad", new GroundLoadCommand(beltSubsystem, intakeSubsystem,
                                assemblySubsystem, flyWheelSubsystem));
                NamedCommands.registerCommand("AssemblyShooting", new AssemblyShootingCommand(assemblySubsystem));
                NamedCommands.registerCommand("Belt", new BeltCommand(beltSubsystem));
                NamedCommands.registerCommand("DownBeltCommand", new DownBeltCommand(beltSubsystem));
                NamedCommands.registerCommand("ReadytoShoot",
                                new ReadyToShootCommand(flyWheelSubsystem, assemblySubsystem));

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
                // .toggleOnTrue(new GroundLoadCommand(beltSubsystem, intakeSubsystem,
                // assemblySubsystem, flyWheelSubsystem));

                new JoystickButton(supplementalController, XboxController.Button.kY.value)
                                .onTrue(new AlignCommand(m_robotDrive, visionSubsystem));

                new JoystickButton(supplementalController, XboxController.Button.kB.value)
                                .toggleOnTrue(new ReadyToShootCommand(flyWheelSubsystem, assemblySubsystem));

                // new JoystickButton(supplementalController,XboxController.Button.kX.value)
                // .whileTrue(new ShootCommand(beltSubsystem, flyWheelSubsystem,
                // assemblySubsystem));

                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(new BeltCommand(beltSubsystem));

                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(new DownAssemblyCommand(assemblySubsystem));

                // new JoystickButton(supplementalController,XboxController.Button.kX.value)
                // .whileTrue(new AssemblyShootingCommand(assemblySubsystem));

                // new JoystickButton(supplementalController, XboxController.Button.kY.value)
                // .whileTrue(new AssemblyIntakeCommand(assemblySubsystem));

                new POVButton(supplementalController, 0)
                                .whileTrue(new AssemblyCommand(assemblySubsystem));

                new JoystickButton(supplementalController, XboxController.Button.kRightBumper.value)
                                .whileTrue(new GroundLoadCommand(beltSubsystem, intakeSubsystem, assemblySubsystem,
                                                flyWheelSubsystem));

                new JoystickButton(supplementalController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new ReverseBeltCommand(beltSubsystem));

                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .toggleOnTrue(new AmpReadyCommand(slideSubsystem, assemblySubsystem));

               new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                  .toggleOnTrue(new SlideCommand(slideSubsystem));

               new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                  .toggleOnTrue(new DownSlideCommand(slideSubsystem));   

                new JoystickButton(buttonBox, 1)
                                .whileTrue(new AssemblyCommand(assemblySubsystem));

                new JoystickButton(buttonBox, 2)
                                .whileTrue(new DownAssemblyCommand(assemblySubsystem));

                new JoystickButton(buttonBox, 3)
                                .toggleOnTrue(new AssemblyShootingCommand(assemblySubsystem));

                new JoystickButton(buttonBox, 4)
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

    public Command getAutonomousCommand() {
        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        //     AutoConstants.kMaxSpeedMetersPerSecond,
        //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //     // Add kinematics to ensure max speed is actually obeyed
        //     .setKinematics(DriveConstants.kDriveKinematics);
   
        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(  
        //         // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // List.of(
        //   new Translation2d(1, 0),
        //   new Translation2d(2, 0)
        // ),
        //     // End 3 meters straight ahead of where we started, facing forward
        //     new Pose2d(3, 0, new Rotation2d(0)),
        //     config);
        
   
        // var thetaController = new ProfiledPIDController(
        //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);
   
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //     exampleTrajectory,
        //     m_robotDrive::getPose, // Functional interface to feed supplier
        //     DriveConstants.kDriveKinematics,
   
        //     // Position controllers
        //     new PIDController(AutoConstants.kPXController, 0, 0),
        //     new PIDController(AutoConstants.kPYController, 0, 0),
        //     thetaController,
        //     m_robotDrive::setModuleStates,
        //     m_robotDrive);
   
            

   
        // Trajectory shootingtrajectory = TrajectoryGenerator.generateTrajectory(  
        //         // Start at the origin facing the +X direction
        // new Pose2d(0, 0, new Rotation2d(0)),
        // List.of(
        //   new Translation2d(1, 0),
        //   new Translation2d(2, 0)
        // ),
        //     // End 3 meters straight ahead of where we started, facing forward
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     config);

        // return new SequentialCommandGroup(
        //         swerveControllerCommand
        // );
        

            // Run path following command, then stop at the end.
         return new SequentialCommandGroup(
         //   new ReadyToShootCommand(flyWheelSubsystem, assemblySubsystem).withTimeout(4)
         //   .alongWith(new SequentialCommandGroup(
         //        new WaitCommand(1),
         //        new BeltCommand(beltSubsystem).withTimeout(2)
         //   )),
            new WaitCommand(11.5),
           
           new DriveUntilDistanceCommand(m_robotDrive, 2)

        //    new DriveUntilDistanceCommand(m_robotDrive, 0.5, true, true)
        //    .alongWith(new GroundLoadCommand(beltSubsystem, intakeSubsystem, assemblySubsystem, flyWheelSubsystem).withTimeout(1)),
           
        //    new DriveUntilDistanceCommand(m_robotDrive, 2.1, false, false)
        //    .alongWith(new ),

        //    new ReadyToShootCommand(flyWheelSubsystem, assemblySubsystem).withTimeout(4)
        //    .alongWith(new SequentialCommandGroup(
        //         new WaitCommand(1),
        //         new BeltCommand(beltSubsystem).withTimeout(2)
        //    ))
           );

        // );
       
      }
}