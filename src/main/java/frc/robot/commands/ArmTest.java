// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;

public class ArmTest extends CommandBase {
  private Shoulder shoulder;
  private Elbow elbow; 
  private double position;

  /** Creates a new ArmTest. */
  public ArmTest(Shoulder shoulder, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoulder = shoulder;
    this.position = position;
    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulder.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setSetpoint(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulder.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
