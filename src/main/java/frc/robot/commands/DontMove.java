// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;

public class DontMove extends CommandBase {
  /** Creates a new DontMove. */
  Drivetrain drivetrain;
  Shifter shifter;
  double lPos;
  double rPos;

  public DontMove(Drivetrain drivetrain, Shifter shifter) {
    this.drivetrain = drivetrain; 
    this.shifter = shifter;
    addRequirements(drivetrain, shifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //drivetrain.resetEncoders();
    lPos = drivetrain.getLeftDistance();
    rPos = drivetrain.getRightDistance();
    shifter.setLowGear();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setLeftDrive(-(drivetrain.getLeftDistance() - lPos));
    drivetrain.setRightDrive(-(drivetrain.getRightDistance() - rPos));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
