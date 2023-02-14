// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Armzies extends SubsystemBase {
  /** Creates a new Armzies. */

  private WPI_TalonSRX shoulder;
  private WPI_TalonSRX elbow;
  private WPI_TalonSRX wrist; 

  private DutyCycleEncoder shoEnc; // shoulder encoder
  private DutyCycleEncoder elEnc; // elbow encoder 
  private DutyCycleEncoder wrEnc; // wrist encoder 

  public Armzies() {
    shoulder = new WPI_TalonSRX(7);
    elbow = new WPI_TalonSRX(8);
    wrist = new WPI_TalonSRX(9);

    shoEnc = new DutyCycleEncoder(0);
    elEnc = new DutyCycleEncoder(1);
    wrEnc = new DutyCycleEncoder(2);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("pos", shoEnc.getAbsolutePosition() * 360);  // see what all positions are :D
    SmartDashboard.putNumber("pos", elEnc.getAbsolutePosition() * 360);
    SmartDashboard.putNumber("pos", wrEnc.getAbsolutePosition() * 360);

  }

  public void moveShoulder(double speed){
    shoulder.set(speed);
  }

  public void moveElbow(double speed){
    elbow.set(speed);
  }

  public void moveWrist(double speed){
    wrist.set(speed);
  }

}
