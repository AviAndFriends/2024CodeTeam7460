package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class SupplementalControllerCommands extends Command {
    int s_controllerPort = 0;

  @Override
  public void initialize() {
    XboxController s_controller = new XboxController(s_controllerPort);
  }

  @Override
  public void execute() {
    new JoystickButton(s_controller, Button.kX.value)
    .whileTrue(new HalveDriveSpeed(m_robotDrive));

    // Shooter is
    // Intake is
    // Assembly is
    // Slide is
    // Belt is
    // Rotate Assembly is

  }
}
