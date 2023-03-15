// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDMotorGroup extends PIDSubsystem implements MotorController{
  private MotorControllerGroup motors;

  private double kpLow; // proportional part of PID loop
  private double kpHigh;

  private double SPEED_LIMIT_HIGH; 
  private double SPEED_LIMIT_LOW;

  private Encoder encoder;

  private SimpleMotorFeedforward ffLow;
  private SimpleMotorFeedforward ffHigh;

  private boolean inLowGear;
  private boolean manualOverride;

  private double outputVoltage;

  //debugging variables
  //private double PIDOutput = 0;
  

  /** Creates a new PIDMotorGroup. */
  public PIDMotorGroup(MotorControllerGroup motors, double maxVelocityLow, double ksLow, Encoder encoder, double kpLow, 
      double maxVelocityHigh, double ksHigh, double kpHigh, 
      double SPEED_LIMIT_LOW, double SPEED_LIMIT_HIGH) {

    super(
        // The PIDController used by the subsystem
        new PIDController(kpHigh, 0, 0));
          this.motors = motors;
          this.encoder = encoder;
          this.kpLow = kpLow;
          this.kpHigh = kpHigh;
          this.inLowGear = false;
          this.manualOverride = false;
          this.SPEED_LIMIT_LOW = SPEED_LIMIT_LOW;
          this.SPEED_LIMIT_HIGH = SPEED_LIMIT_HIGH;
          enable();

          ffLow = new SimpleMotorFeedforward(ksLow, 12.0/maxVelocityLow);
          ffHigh = new SimpleMotorFeedforward(ksHigh, 12.0/maxVelocityHigh);
          
  }
  /*public double getPIDOutput(){//debugging method
    return PIDOutput;
  }*/

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    //PIDOutput = output;

    if(inLowGear) {
      outputVoltage = output + ffLow.calculate(setpoint);
      motors.setVoltage(outputVoltage);
    } 
    else {
      outputVoltage = output + ffHigh.calculate(setpoint);
      motors.setVoltage(outputVoltage);
    }
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getRate();
  }

  @Override
  public void set(double speed) {
    if(manualOverride) {
      motors.setVoltage(speed * 12);
    } 
    else if(inLowGear) {
      setSetpoint(speed * SPEED_LIMIT_LOW);
    }
    else {
      setSetpoint(speed * SPEED_LIMIT_HIGH);
    }
  }

  @Override
  public double get() {
    return motors.get();
  }

  @Override
  public void setInverted(boolean isInverted) {
    motors.setInverted(isInverted);
  }

  @Override
  public boolean getInverted() {
    return motors.getInverted();
  }

  @Override
  public void stopMotor() {
    if(manualOverride) {
      motors.stopMotor();
    } 
    else {
      setSetpoint(0);
    }
  }

  public void shiftGear(boolean inLowGear) {
    this.inLowGear = inLowGear;

    if(inLowGear) {
      super.getController().setP(kpLow);
    }
    else {
      super.getController().setP(kpHigh);
    }
  }
  
  public void setManualOverride(boolean manualOverride) {
    this.manualOverride = manualOverride;

    if(manualOverride) {
      disable();
    } else {
      enable();
    }
  }

}
