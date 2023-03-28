// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmControlCommand;
import frc.robot.commands.ArmControlCommandcopy;
import frc.robot.commands.AutonDriveAndPlaceCommand;
import frc.robot.commands.AutonDriveCommand;
import frc.robot.commands.ClawInCommand;
import frc.robot.commands.ClawOutCommad;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.LightsCommand;
import frc.robot.commands.LockButtonCommand;
import frc.robot.commands.ToggleDriveSpeed;
import frc.robot.commands.ZreoGyroCommmand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.test;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final LightsSubsystem m_lights = new LightsSubsystem();
  private AutonDriveCommand m_AutonDriveCommand ;
  private AutonDriveAndPlaceCommand m_DriveAndPlaceCommand;
  private LightsCommand m_lightsCommand;
  private ArmControlCommand m_ArmControlCommand;


  private final Joystick m_controller = new Joystick(0);
  private final Joystick m_clawController = new Joystick(2);
  private final Joystick m_clawController2 = new Joystick(3);


  private final XboxController m_xbox = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    m_ArmControlCommand = new ArmControlCommand(m_ArmSubsystem,() -> -m_clawController2.getY(),
     /* */ () -> m_xbox.getLeftY() /*meaningless rn */,() -> -m_clawController.getY(),() -> -m_clawController.getX());
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

   /*  m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND ,
           () -> -modifyAxis(m_controller.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getTwist()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND*0.25
    )); */

    m_lightsCommand = new LightsCommand(m_lights);
    m_ArmSubsystem.setDefaultCommand(m_ArmControlCommand);
   // m_lights.setDefaultCommand(m_lightsCommand);

   m_AutonDriveCommand = new AutonDriveCommand(m_drivetrainSubsystem);
   m_DriveAndPlaceCommand = new AutonDriveAndPlaceCommand(m_drivetrainSubsystem, m_ArmSubsystem);

    // Configure the button bindings
  }

  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    //new Button(m_controller::getBackButton) //**************** */
    final JoystickButton zeroGyro = new JoystickButton(m_controller, 4) ;
            // No requirements because we don't need to interrupt anything
            zeroGyro.onTrue(new ZreoGyroCommmand(m_drivetrainSubsystem));  //******* */
     // new JoystickButton(m_controller, 3).(m_drivetrainSubsystem.toggleSlowMode(););
    // final JoystickButton lightsButton = new JoystickButton(m_xbox, 2);
    // final JoystickButton armSingleButton = new JoystickButton(m_xbox, 1);
     final JoystickButton clawInButton = new JoystickButton(m_clawController2, 2);
     final JoystickButton clawOutButton = new JoystickButton(m_clawController2, 3);

    //final JoystickButton driveLockButton = new JoystickButton(m_controller, 4);
   //  armSingleButton.onTrue(new ArmControlCommandcopy(m_ArmSubsystem));
   // final test m_testVariable = new test(null, clawOutButton);
   //  m_testVariable.whileItIsTrue(new ClawInCommand(m_ArmSubsystem));
   clawInButton.whileTrue(new ClawInCommand(m_ArmSubsystem));
     clawOutButton.whileTrue(new ClawOutCommad(m_ArmSubsystem));

  //  driveLockButton.onTrue(new LockButtonCommand(m_drivetrainSubsystem));

    // lightsButton.onTrue(new LightsCommand(m_lights));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_DriveAndPlaceCommand;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.2);
    //first 0.05

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
