// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shifter extends SubsystemBase {
  private Solenoid shifter;
  private Drivetrain drivetrain; 
  private final double SHIFT_POINT_HIGH = 4; // TODO it's just a todo. we know what we have to do ;)
  private final double SHIFT_POINT_LOW = 3.0;
  /** Creates a new Shifter. */
  public Shifter(Drivetrain drivetrain) {
    shifter = new Solenoid(PneumaticsModuleType.REVPH, 0);
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void autoShift(){
    if(isLowGear()&&(Math.abs(drivetrain.getLeftSpeed()) > SHIFT_POINT_HIGH || Math.abs(drivetrain.getRightSpeed()) > SHIFT_POINT_HIGH )){
      shifter.set(false);
    }//shift to high
    else if(!isLowGear()&&(Math.abs(drivetrain.getLeftSpeed()) < SHIFT_POINT_LOW && Math.abs(drivetrain.getRightSpeed()) < SHIFT_POINT_LOW )){
      shifter.set(true);
    }//shift to low
  }

  public boolean isLowGear(){
    return shifter.get();
  }

  public void setLowGear(){
    drivetrain.shiftGear(true);
    shifter.set(true);
  }

  public void setHighGear(){
    drivetrain.shiftGear(false);
    shifter.set(false);
  }

  public Command setGear(boolean low){
    if(low){
      drivetrain.shiftGear(true);
      return this.run(() -> shifter.set(true));
    }
    else {
      drivetrain.shiftGear(false);
      return this.run(() -> shifter.set(false));
    }
  }
}
