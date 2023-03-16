// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class ArmMovementCommand extends CommandBase {
  private final double[] PARKED_POSITION = {-90 , 0 , 0};//TODO need parked numbers
  private final double S_PREP_FOR_PARK = -180;//the position the shoulder needs to be in before the rest can fold in, should only need it for one side
  private Shoulder shoulder;
  private Elbow elbow;
  private Wrist wrist;
  private double[] positionList; //shoulder, elbow, wrist
  private boolean changingSides;
  /** Creates a new ArmMovementCommand. */
  public ArmMovementCommand(Shoulder shoulder , Elbow elbow , Wrist wrist , double[] positionList) {
    this.shoulder = shoulder;
    this.elbow = elbow;
    this.wrist = wrist;
    this.positionList = positionList;
    /*if((shoulder.getMeasurement() < 0 && positionList[0] < 0) || (shoulder.getMeasurement() >= 0 && positionList[0] >= 0)){
      changingSides = false;
    }
    else{
      changingSides = true;
    }*/
    changingSides = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(changingSides /*|| Arrays.equals(PARKED_POSITION , positionList)*/){
      if(Math.abs(S_PREP_FOR_PARK - shoulder.getMeasurement()) > shoulder.getController().getPositionTolerance() && shoulder.getMeasurement() < -90){
        shoulder.setSetpoint(S_PREP_FOR_PARK);
      }//TODO assume parked arm faces forward andf forward is the positive angle side
      /*else if(Math.abs(PARKED_POSITION[2] - wrist.getMeasurement()) > wrist.getController().getPositionTolerance()){
        wrist.setSetpoint(PARKED_POSITION[2]);
      }
      else if(Math.abs(PARKED_POSITION[1] - elbow.getMeasurement()) > elbow.getController().getPositionTolerance()){
        elbow.setSetpoint(PARKED_POSITION[1]);
      }*/
      else if(Math.abs(PARKED_POSITION[0] - shoulder.getMeasurement()) > shoulder.getController().getPositionTolerance()){
        shoulder.setSetpoint(PARKED_POSITION[0]);
      }
      else{
        changingSides = false;
      }
    }
    else{
      if(Math.abs(positionList[0] - shoulder.getMeasurement()) > shoulder.getController().getPositionTolerance()){
        shoulder.setSetpoint(positionList[0]);
      }
      /*else if(Math.abs(positionList[1] - elbow.getMeasurement()) > elbow.getController().getPositionTolerance()){
        elbow.setSetpoint(positionList[1]);
      }
      else if(Math.abs(positionList[2] - wrist.getMeasurement()) > wrist.getController().getPositionTolerance()){
        wrist.setSetpoint(positionList[2]);
      }*/

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shoulder.getController().atSetpoint() /*&& elbow.getController().atSetpoint() && wrist.getController().atSetpoint()*/;
  }
}
