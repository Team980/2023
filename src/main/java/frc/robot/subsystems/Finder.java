// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Finder extends SubsystemBase {
  /** Creates a new Finder. */
  private Pixy2 pixy;
  private int coneCount;
  private int sig = Pixy2CCC.CCC_SIG1;
  private ArrayList<Block> cones;

  public Finder() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init();
    cones = new ArrayList<Block>();
  }

  @Override
  public void periodic() {
    coneCount = pixy.getCCC().getBlocks(false, sig, 11);
    SmartDashboard.putNumber("targets found", coneCount);
    cones = pixy.getCCC().getBlockCache();
  }

  public int[] findClosestCargo(){
    int[] dize = new int[2];//[0]-157 to 157
    if(coneCount == 0){
      dize[0] = 160;//designated did not find a ball
      return dize;
    }
    Block closestCone = cones.get(0);
    for(int index = 1; index<cones.size(); index++){
      if(cones.get(index).getWidth()>closestCone.getWidth()){
        closestCone = cones.get(index);
      }//end if
    }//end for
    dize[0]=closestCone.getX()-157;//TODO get offset for grabbing gamepiece
    dize[1]=closestCone.getWidth();
    SmartDashboard.putNumber("largest x location", dize[0]);
    SmartDashboard.putNumber("largest width", dize[1]);
    return dize;
  }//end findClosestCargo

  public void setItem(boolean isCone){
    if(isCone){
      sig = Pixy2CCC.CCC_SIG1;//cone
    }
    else{
      sig = Pixy2CCC.CCC_SIG2;//cube
    }
  }
}//end subsystem
