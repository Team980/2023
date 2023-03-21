// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Shoulder;

public class ArmCommand2 extends CommandBase {
  //removing wrist values, wrist will use the auto stay level command elsewhere and switching sides will be its own command
  private final double[] PARKED_POSITION_F = {-90 , 160};//TODO all numbers are estimates needing real values
  private final double[] SCORE_HIGH_F = {-20, 30};
  private final double[] SCORE_MID_F = {-45, 90};
  private final double[] FLOOR_F = {-45, 0};
  private final double[] H_STATION_F = {-80, 150};

  private final double[] PARKED_POSITION_R = {-90 , -160};
  private final double[] SCORE_HIGH_R = {-160, -30};
  private final double[] SCORE_MID_R = {-135, -90};
  private final double[] FLOOR_R = {-135, 0};
  private final double[] H_STATION_R = {-100, -150};

  private final double[][] All_POSITIONS = {PARKED_POSITION_F , FLOOR_F  , SCORE_MID_F , SCORE_HIGH_F , H_STATION_F , 
    PARKED_POSITION_R , FLOOR_R , SCORE_MID_R , SCORE_HIGH_R , H_STATION_R};

  private final double S_PREP_FOR_SWITCH_F = 0; //TODO find lowest position to safely flip sides
  private final double S_PREP_FOR_SWITCH_R = -180;

  private Shoulder shoulder;
  private Elbow elbow;
  //private Wrist wrist;
  private int position; //0-parked, 1-floor, 2-mid, 3-high, 4-h_station, 5-switch
  private double[] rPos;
  private boolean inFront;
  private boolean finished;
  private double prepSwitch;

  /** Creates a new ArmCommand2. */
  public ArmCommand2(Shoulder shoulder , Elbow elbow , int position) {
    this.shoulder = shoulder;
    this.elbow = elbow;
    //this.wrist = wrist;
    this.position = position;
    inFront = true;
    if(inFront){
      rPos = All_POSITIONS[position];
      prepSwitch = S_PREP_FOR_SWITCH_F;
    }
    else{
      prepSwitch = S_PREP_FOR_SWITCH_R;
      if(position <= 4){
        rPos = All_POSITIONS[position + 5];
      }
      else{
        rPos = All_POSITIONS[0];//front parked
      }
      
    }
    finished = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder , elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(position == 5){
      if(Math.abs(prepSwitch - shoulder.getMeasurement()) > 5){
        shoulder.setSetpoint(prepSwitch);
      }
      else if(Math.abs(rPos[1] - elbow.getMeasurement()) > 5){
        elbow.setSetpoint(rPos[1]);
      }
      else{
        shoulder.setSetpoint(rPos[0]);
        finished = true;
      }
    }
    else{
      shoulder.setSetpoint(rPos[0]);
      elbow.setSetpoint(rPos[1]);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
