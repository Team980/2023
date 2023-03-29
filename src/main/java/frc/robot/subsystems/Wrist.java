// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

public class Wrist extends PIDSubsystem {
  /** Creates a new Wrist. */
  private ArmSensors sensors;

  private final double KS = 0.1;
  private final double GEAR_RATIO = 100;
  private final double POSITION_TOLERANCE = 1;

  private DoubleSolenoid wheelyGrab;

  private WPI_TalonSRX wrist; 

  public Wrist(ArmSensors sensors) {
    super(
        // The PIDController used by the subsystem
        new PIDController(12.0 / 80, 0, 0));

        wrist = new WPI_TalonSRX(10); // 8 is now the other shoulder, this will need to be a new talon with a new ID
        
        this.sensors = sensors;
        super.getController().setTolerance(POSITION_TOLERANCE);
        wrist.setNeutralMode(NeutralMode.Brake);
        wrist.setInverted(false);
        wheelyGrab = new DoubleSolenoid(PneumaticsModuleType.REVPH, 9, 14);
        enable();
        // setSetpoint(-(sensors.getShoulderAngle() + sensors.getElbowAngle())); // 90
  }

  public void runWrist(double speed) {
    wrist.set(speed);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here 
    //if(!sensors.getSCon() && !sensors.getWCon()) {
      wrist.setVoltage(output + customFFCalc(setpoint));
    //}

      SmartDashboard.putNumber("W_PIDOut", output);
      SmartDashboard.putNumber("W_FF", customFFCalc(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    // setSetpoint(-sensors.getShoulderAngle());
    return sensors.getWristAngle();
  }

  public double customFFCalc(double goalPosition) { //direction is either 1 or -1 depending on the sensor
    double qs = sensors.getShoulderAngle();
    // double qe = sensors.getElbowAngle();
    double qw = sensors.getWristAngle();

    double csew = Math.cos(Math.toRadians(qs /*+ qe*/ + qw));
    
    double gWrist = csew * W_SEGMENT_MASS * W_CG_FROM_JOINT;

    return KS * Math.signum(goalPosition - qw) + (12 * gWrist / (BAG_MOTOR_STALL_TORQUE * GEAR_RATIO));//-12 change to voltage and oppose gravity
  }

  public void kindaManual(double move) {
    if(Math.abs(move) > 0.2) {
      setSetpoint(0.5 * move + getSetpoint());
    }
  }

  public CommandBase holdPosition(){
    return this.runOnce(() -> setSetpoint(90));
  }

  public CommandBase open(boolean openGrab) {
    if(openGrab) {
      return this.runOnce(() -> wheelyGrab.set(Value.kForward));
    }
    else {
      return this.runOnce(() -> wheelyGrab.set(Value.kReverse));
    }
  }

  public CommandBase horizAuto(){
    /*if(sensors.getElbowAngle() >= -90){
      return this.runOnce(() -> setSetpoint(-(sensors.getShoulderAngle() + sensors.getElbowAngle())));
    }
    else{
      return this.runOnce(() -> setSetpoint(-180 - (sensors.getShoulderAngle() + sensors.getElbowAngle())));

    }*/

    return this.runOnce(() -> setSetpoint(-(sensors.getShoulderAngle() + sensors.getElbowAngle())));
  }
}
