// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Wrist extends PIDSubsystem {
  /** Creates a new Wrist. */

  private ArmFeedforward wrFF; 

  private WPI_TalonSRX wrist; 
  private DutyCycleEncoder wrEnc; // wrist encoder 

  public Wrist() {
    super(
        // The PIDController used by the subsystem
        new PIDController(12.0 / 90, 0, 0));

        wrFF = new ArmFeedforward(0, 0, 0);

        wrist = new WPI_TalonSRX(9);
        wrEnc = new DutyCycleEncoder(2);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here

    wrist.setVoltage(output + wrFF.calculate(setpoint, 15));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    double encVal = wrEnc.getAbsolutePosition() * 360;

    SmartDashboard.putNumber("wrPos", encVal);

    return encVal;
  }
}
