// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSensors extends SubsystemBase {
  /** Creates a new ArmSensors. */
  private DutyCycleEncoder shoulder;
  private DutyCycleEncoder elbow;
  private DutyCycleEncoder wrist;

  private boolean sConnect;
  private boolean eConnect;
  private boolean wConnect;

  public ArmSensors() {
    shoulder = new DutyCycleEncoder(4); // 4
    elbow = new DutyCycleEncoder(5); // 5
    wrist = new DutyCycleEncoder(6); // 6
  }

  @Override
  public void periodic() {
    sConnect = shoulder.isConnected();
    SmartDashboard.putBoolean("s con", sConnect);
    eConnect = elbow.isConnected();
    SmartDashboard.putBoolean("e con", eConnect);
    wConnect = wrist.isConnected();
    SmartDashboard.putBoolean("w con", wConnect);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shoPos", getShoulderAngle());  // see what all positions are :D
    SmartDashboard.putNumber("elPos", getElbowAngle());
    SmartDashboard.putNumber("wriPos", getWristAngle());
  }

  //all need offsets for 90 parallel to ground when straight out
  public double getShoulderAngle(){
    return shoulder.getAbsolutePosition() * 360 - 223.5; // 221.5 
  }

  public double getElbowAngle(){
    return elbow.getAbsolutePosition() * 360 - 183.6; // 183.6
  }

  public double getWristAngle(){
    return (wrist.getAbsolutePosition() * 360 - 177.5) * -1; // 143
  }

  public boolean getSCon(){
    return sConnect;
  }
  public boolean getECon(){
    return eConnect;
  }
  public boolean getWCon(){
    return wConnect;
  }
}
