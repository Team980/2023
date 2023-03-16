// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmMovementCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmSensors;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shifter shifter  = new Shifter (drivetrain);
  private final ArmSensors armSensors = new ArmSensors();
  private final Shoulder shoulder = new Shoulder(armSensors);
  private final Wrist wrist = new Wrist(armSensors);
  private final Elbow elbow = new Elbow(armSensors);

  private final CommandXboxController xbox = new CommandXboxController(2);

  private final CommandJoystick wheel = new CommandJoystick(0);
  private final CommandJoystick throttle = new CommandJoystick(1);
  private final CommandJoystick prajBox = new CommandJoystick(4);

  private final double[] test0 = {0, 0, 0};
  private final double[] test1 = {-180, 0, 0};
  private final double[] test2 = {-90, 0, 0};
 /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrain.setDefaultCommand(Commands.run(
      () -> drivetrain.driveRobot(throttle.getY(), wheel.getX()), 
      drivetrain
      )); 

    shifter.setDefaultCommand(shifter.setGear(true));

    shoulder.setDefaultCommand(Commands.run(
      () -> shoulder.kindaManual(-xbox.getLeftY()),
      shoulder
       ));

    /*elbow.setDefaultCommand(Commands.run(
      () -> elbow.runElbow(-xbox.getRightY()),
      elbow
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
    xbox.b().onTrue(Commands.runOnce(
      () -> elbow.setSetpoint(75),
    elbow
  ));
    /*xbox.x().onTrue(Commands.runOnce(
      () -> elbow.setSetpoint(-90),
    elbow
  ));*/
    xbox.a().onTrue(Commands.runOnce(
      () -> elbow.setSetpoint(165),
    elbow
  ));

  /*xbox.b().onTrue(new ArmMovementCommand(shoulder, elbow, wrist, test0));
  xbox.x().onTrue(new ArmMovementCommand(shoulder, elbow, wrist, test1));
  xbox.a().onTrue(new ArmMovementCommand(shoulder, elbow, wrist, test2)); */

  throttle.button(4).onTrue(shifter.setGear(false));
  throttle.button(3).onTrue(shifter.setGear(true));

  xbox.rightBumper().onTrue(wrist.open(true));
  xbox.leftBumper().onTrue(wrist.open(false));

  // xbox.start().onTrue(new InstantCommand(drivetrain::enableManualOverride, drivetrain));
  // xbox.back().onTrue(new InstantCommand(drivetrain::disableManualOverride, drivetrain));

    /* Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));*/

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
