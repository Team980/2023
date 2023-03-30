// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmCommand2;
import frc.robot.commands.ArmMovementCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DontMove;
import frc.robot.commands.DriveOutAuto;
import frc.robot.subsystems.ArmSensors;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final PowerDistribution pdh = new PowerDistribution();
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain(pdh);
  private final Shifter shifter  = new Shifter (drivetrain);
  private final ArmSensors armSensors = new ArmSensors();
  private final Shoulder shoulder = new Shoulder(armSensors);
  private final Wrist wrist = new Wrist(armSensors);
  private final Elbow elbow = new Elbow(armSensors);

  private final CommandXboxController xbox = new CommandXboxController(2);

  private final CommandJoystick wheel = new CommandJoystick(0);
  private final CommandJoystick throttle = new CommandJoystick(1);
  private final CommandJoystick prajBox = new CommandJoystick(4);


 /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drivetrain.setDefaultCommand(Commands.run(
      () -> drivetrain.driveRobot(-throttle.getY(), -wheel.getX()), 
      drivetrain
      )); 

    // shifter.setDefaultCommand(shifter.setGear(true));

    shoulder.setDefaultCommand(Commands.run(
    () -> shoulder.runShoulder(-xbox.getLeftY()),
    shoulder
      ));

    elbow.setDefaultCommand(Commands.run(
    () -> elbow.runElbow(-xbox.getRightY()),
    elbow
      ));

    wrist.setDefaultCommand(wrist.horizAuto());

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
    xbox.back().onTrue(Commands.parallel(shoulder.holdPosition() , wrist.holdPosition()));//will stop the arm and clear running commands
    xbox.a().onTrue(new ArmCommand2(shoulder , elbow , 1));//floor
    xbox.b().onTrue(new ArmCommand2(shoulder , elbow , 2));//mid
    xbox.y().onTrue(new ArmCommand2(shoulder , elbow , 3));//high
    xbox.x().onTrue(new ArmCommand2(shoulder , elbow , 0));//park
    xbox.povRight().onTrue(new ArmCommand2(shoulder , elbow , 4));//human station
    //xbox.start().onTrue(new ArmMovementCommand(shoulder , elbow , wrist , 5));//switch sides

    xbox.rightBumper().onTrue(wrist.open(true));
    xbox.leftBumper().onTrue(wrist.open(false));

    xbox.povLeft().onTrue(Commands.run(wrist::parkWrist, wrist));
    xbox.povRight().onTrue(wrist.horizAuto());

    /*xbox.b().onTrue(Commands.run(
      () -> wrist.runWrist(-xbox.getRightY()),
      wrist
       ));

    xbox.a().onTrue(Commands.runOnce(() -> wrist.setSetpoint(-armSensors.getShoulderAngle()), wrist));*/
    prajBox.button(1).whileTrue(new DontMove(drivetrain, shifter));
  
    /*xbox.b().onTrue(Commands.runOnce(
      () -> elbow.setSetpoint(75),
    elbow
  ));
    xbox.x().onTrue(Commands.runOnce(
      () -> elbow.setSetpoint(-90),
    elbow
  ));
    xbox.a().onTrue(Commands.runOnce(
      () -> elbow.setSetpoint(165),
    elbow
  ));*/

  /*throttle.button(3).onTrue(shifter.setGear(false));
  throttle.button(4).onTrue(shifter.setGear(true));*/
  throttle.button(4).onTrue(Commands.runOnce(shifter::setHighGear, shifter));
  throttle.button(3).onTrue(Commands.runOnce(shifter::setLowGear, shifter));

  //prajBox.button(0).onTrue(drivetrain.reverseFront(true)).onFalse(drivetrain.reverseFront(false));
  throttle.povUp().onTrue(Commands.runOnce(drivetrain::reverseFrontToggle, drivetrain));
  throttle.povUpLeft().onTrue(Commands.runOnce(drivetrain::reverseFrontToggle, drivetrain));
  throttle.povUpRight().onTrue(Commands.runOnce(drivetrain::reverseFrontToggle, drivetrain));
  throttle.povLeft().onTrue(Commands.runOnce(drivetrain::reverseFrontToggle, drivetrain));
  throttle.povRight().onTrue(Commands.runOnce(drivetrain::reverseFrontToggle, drivetrain));

  


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
    return new DriveOutAuto(drivetrain);
  }
}
