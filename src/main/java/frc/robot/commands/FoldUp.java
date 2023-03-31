// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSensors;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class FoldUp extends CommandBase {
  /** Creates a new FoldUp. */

  private Shoulder shoulder;
  private Wrist wrist;
  private Elbow elbow;
  private ArmSensors sensors;
  private boolean finSho;
  private boolean finEl;


  public FoldUp(Shoulder shoulder, Elbow elbow, Wrist wrist, ArmSensors sensors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.elbow = elbow;
    this.sensors = sensors;
    finSho = false;
    finEl = false;
    addRequirements(shoulder, elbow, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.horizAuto();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(sensors.getElbowAngle() < 130){
      elbow.runElbow(0.5);
    }
    else{
      elbow.runElbow(0);
      finEl = true;
      if(sensors.getShoulderAngle() > -80){
        shoulder.runShoulder(-0.5);
      }
      else{
        shoulder.runShoulder(0);
        finSho = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finSho && finEl;
  }
}
