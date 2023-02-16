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

public class Elbow extends PIDSubsystem {
  /** Creates a new Elbow. */

  private ArmFeedforward elFF; 

  private WPI_TalonSRX elbow;
  private DutyCycleEncoder elEnc; // elbow encoder

  public Elbow() {
    super(
        // The PIDController used by the subsystem
        new PIDController(12.0 / 90, 0, 0));

        elFF = new ArmFeedforward(0, 0, 0);  // TODO find constants 

        elbow = new WPI_TalonSRX(8);
        elEnc = new DutyCycleEncoder(1);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here

    elbow.setVoltage(output + elFF.calculate(setpoint, 15));  // TODO figure out how fast we want it to move
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    double encVal = elEnc.getAbsolutePosition() * 360;

    SmartDashboard.putNumber("elPos", encVal);

    return encVal;
  }
}
