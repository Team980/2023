// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

public class Wrist extends PIDSubsystem {
  /** Creates a new Wrist. */
  private ArmSensors sensors;

  private final double KS = .05;
  private final double GEAR_RATIO = 100;
  private final double POSITION_TOLERANCE = 5;

  private WPI_TalonSRX wrist; 

  public Wrist(ArmSensors sensors) {
    super(
        // The PIDController used by the subsystem
        new PIDController(12.0 / 90, 0, 0));

        wrist = new WPI_TalonSRX(8);
        this.sensors = sensors;
        super.getController().setTolerance(POSITION_TOLERANCE);
        //enable();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here 
    wrist.setVoltage(output + customFFCalc(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return sensors.getWristAngle();
  }

  public double customFFCalc(double goalPosition) { //direction is either 1 or -1 depending on the sensor
    double qs = sensors.getShoulderAngle();
    double qe = sensors.getElbowAngle();
    double qw = sensors.getWristAngle();

    double csew = Math.cos(Math.toRadians(qs + qe + qw));
    
    double gWrist = csew * W_SEGMENT_MASS * W_CG_FROM_JOINT;

    return KS * Math.signum(goalPosition - qw) + (12 * gWrist / (BAG_MOTOR_STALL_TORQUE * GEAR_RATIO));//-12 change to voltage and oppose gravity
  }

  public void kindaManual(double move) {
    if(Math.abs(move) > 0.2) {
      setSetpoint(move + getMeasurement());
    }
  }

}
