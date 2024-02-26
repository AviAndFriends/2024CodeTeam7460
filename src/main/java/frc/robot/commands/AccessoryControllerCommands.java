package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class AccessoryControllerCommands extends Command {
    int a_controllerPort = 0;

  @Override
  public void initialize() {
    XboxController a_controller = new XboxController(a_controllerPort);
  }

  @Override
  public void execute() {
    // Shooter is
    // Intake is
    // Assembly is
    // Slide is
    // Belt is
    // Rotate Assembly is

  }
}
