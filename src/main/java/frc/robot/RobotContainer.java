// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.controller.GameController;
import frc.robot.controller.Xbox;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final GameController m_driver = new GameController(Constants.DRIVER_JOYSTICK_1_USB_ID, new Xbox());

  private final Drive drive = Drive.getInstance();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Pass the driver controller to the drive subsystem for teleop control
    drive.setDriverController(m_driver);

    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    drive.setDefaultCommand(
            // A split-stick arcade command, with forward/backward controlled by the left
            // hand, and turning controlled by the right.
            new RunCommand(
                    () ->
                            drive.drive(
                                    m_driver.getY(GenericHID.Hand.kLeft),
                                    m_driver.getX(GenericHID.Hand.kRight),
                                    m_driver.getX(GenericHID.Hand.kLeft),
                                    false)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
