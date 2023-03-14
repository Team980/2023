// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

public class Shoulder extends PIDSubsystem {
  /** Creates a new Shoulder. */

  private ArmSensors sensors;

  private final double KS = .05;
  private final double GEAR_RATIO = 600;

  private WPI_TalonSRX shoulder;


  public Shoulder(ArmSensors sensors) {
    super(
        // The PIDController used by the subsystem
        new PIDController(12.0 / 90, 0, 0));

        shoulder = new WPI_TalonSRX(11);
        this.sensors = sensors;
        //enable();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here

    shoulder.setVoltage(output + customFFCalc(setpoint));

  }

  public void runShoulder(double speed) {
    shoulder.set(speed);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return sensors.getShoulderAngle();
  }

  public double customFFCalc(double goalPosition) { //direction is either 1 or -1 depending on the sensor
    double qs = getMeasurement();
    double qe = sensors.getElbowAngle();
    double qw = sensors.getWristAngle();

    double cs = Math.cos(qs);
    double cse = Math.cos(qs + qe);
    double csew = Math.cos(qs + qe + qw);

    double gShoulder = ACCEL_G * (cs * (SHO_SEGMENT_MASS * SHO_CG_FROM_JOINT + EL_SEGMENT_MASS * SHO_SEGMENT_LENGTH + W_SEGMENT_MASS * SHO_SEGMENT_LENGTH) + 
      cse * (EL_SEGMENT_MASS * EL_CG_FROM_JOINT + W_SEGMENT_MASS * EL_SEGMENT_LENGTH) + 
      csew * W_SEGMENT_MASS * W_CG_FROM_JOINT);

    return KS * Math.signum(goalPosition - qs) + (-12 * gShoulder / (BAG_MOTOR_STALL_TORQUE * GEAR_RATIO)); //-12 change to voltage and oppose gravity
}

  public void kindaManual(double move) {
    if(Math.abs(move) > 0.2) {
      setSetpoint(move + getMeasurement());
    }
  }

  public Command setPosition(double degrees){
    return this.runOnce(() -> setSetpoint(degrees));
  }
}
