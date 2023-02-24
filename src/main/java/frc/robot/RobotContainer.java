// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shoulder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shifter shifter  = new Shifter (drivetrain);
  //private final Shoulder shoulder = new Shoulder();

  private final CommandXboxController xbox = new CommandXboxController(2);

  private final CommandJoystick wheel = new CommandJoystick(0);
  private final CommandJoystick throttle = new CommandJoystick(1);
  private final CommandJoystick prajBox = new CommandJoystick(4);


  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrain.setDefaultCommand(new RunCommand(
      () -> drivetrain.driveRobot(xbox.getLeftY(), xbox.getRightX()), 
      drivetrain
      ));
    shifter.setDefaultCommand(new RunCommand(shifter::setLowGear, shifter) );

    /*shoulder.setDefaultCommand(new RunCommand(
      () -> shoulder.kindaManual(xbox.getLeftTriggerAxis()),
      shoulder
       ));*/
       
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    xbox.rightStick().onTrue(new RunCommand(shifter::setHighGear, shifter) );
    xbox.leftStick().onTrue(new RunCommand(shifter::setLowGear, shifter) );
    /*throttle.button(5).onTrue(new RunCommand(
      () -> shifter.autoShift(),
      shifter
      ));*/

    xbox.start().onTrue(new InstantCommand(drivetrain::enableManualOverride, drivetrain));
    xbox.back().onTrue(new InstantCommand(drivetrain::disableManualOverride, drivetrain));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
