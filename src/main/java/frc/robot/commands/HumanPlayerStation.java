// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSensors;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.Wrist;

public class HumanPlayerStation extends CommandBase {
  /** Creates a new ScoreAuto. */
  private Shoulder shoulder;
  private Wrist wrist;
  private Elbow elbow;
  //private Targeting targeting;

  private boolean finSho;
  private boolean finEl;

  private ArmSensors sensors;
  
  public HumanPlayerStation(Shoulder shoulder, Elbow elbow, Wrist wrist, ArmSensors sensors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoulder = shoulder;
    this.wrist = wrist;
    this.elbow = elbow;
    this.sensors = sensors;
    //this.targeting = targeting;
    finSho = false;
    finEl = false;
    addRequirements(shoulder, elbow, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // 3ft 3 in
    if(sensors.getShoulderAngle() < -31){ // 0
      shoulder.runShoulder(0.5);
    }
    else{
      shoulder.runShoulder(0);
      finSho = true;
      wrist.setSetpoint(-75);
      if(sensors.getElbowAngle() > 123){ // 20
        elbow.runElbow(-0.5);
      }
      else{
        elbow.runElbow(0);
        finEl = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finSho && finEl;
  }
}