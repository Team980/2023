// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

public class Shoulder extends PIDSubsystem {
  /** Creates a new Shoulder. */

  private ArmSensors sensors;

  private final double KS = .2;
  private final double GEAR_RATIO = 420; // 600
  private final double POSITION_TOLERANCE = 2; // 5

  private WPI_TalonSRX shoulder;
  private WPI_TalonSRX shoulder2;

  private double currentPosition; 

  private PowerDistribution pdh;

  public Shoulder(ArmSensors sensors, PowerDistribution pdh) {
    super(
        // The PIDController used by the subsystem
        new PIDController(12.0 / 45, 0, 0));


        shoulder = new WPI_TalonSRX(11);
        shoulder2 = new WPI_TalonSRX(8);
        shoulder.setNeutralMode(NeutralMode.Brake);

        shoulder.setInverted(false);
        shoulder2.setInverted(false);
        this.sensors = sensors;
        this.pdh = pdh;
        currentPosition = getMeasurement();
        super.getController().setTolerance(POSITION_TOLERANCE);
        //enable();
        //setSetpoint(currentPosition);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    shoulder.set(Math.signum(output) * .5);
    shoulder2.set(Math.signum(output) * .5);
    // if(sensors.getSCon() && sensors.getWCon()){
      //shoulder.setVoltage(output + customFFCalc(setpoint));
      //shoulder2.setVoltage(output + customFFCalc(setpoint));
  //}

    SmartDashboard.putNumber("S_PIDOut", output);
  }

  public void runShoulder(double speed) {
    if(Math.abs(speed) > 0.1 && pdh.getCurrent(1) < 6 && pdh.getCurrent(8) < 6){
      if(!(sensors.getShoulderAngle() >= 20 && speed > 0)){
        disable();
        shoulder.set(speed);
        shoulder2.set(speed);
        currentPosition = getMeasurement();
      }
    }
    else{
    //  enable();
    //  setSetpoint(currentPosition);
    shoulder.stopMotor();
    shoulder2.stopMotor();
    }
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

    double cs = Math.cos(Math.toRadians(qs));
     double cse = Math.cos(Math.toRadians(qs + qe));
     double csew = Math.cos(Math.toRadians(qs + qe + qw));
    // double csew = Math.cos(Math.toRadians(qs + qw));

    double gShoulder = cs * (SHO_SEGMENT_MASS * SHO_CG_FROM_JOINT + EL_SEGMENT_MASS * SHO_SEGMENT_LENGTH + W_SEGMENT_MASS * SHO_SEGMENT_LENGTH) + 
      cse * (EL_SEGMENT_MASS * EL_CG_FROM_JOINT + W_SEGMENT_MASS * EL_SEGMENT_LENGTH) + 
      csew * W_SEGMENT_MASS * W_CG_FROM_JOINT;

    /*double gShoulder = cs * (SHO_SEGMENT_MASS * SHO_CG_FROM_JOINT + W_SEGMENT_MASS * SHO_SEGMENT_LENGTH) + 
    csew * W_SEGMENT_MASS * W_CG_FROM_JOINT; */

    return KS * Math.signum(goalPosition - qs) + (12 * gShoulder / (DUAL_MOTOR_STALL_TORQUE * GEAR_RATIO)); //12 change to voltage--/* */
    
}

  /*public void kindaManual(double move) {
    if(Math.abs(move) > 0.2) {
      setSetpoint(0.5 * move + getSetpoint());
    }
  }*/

  public CommandBase holdPosition(){
    return this.runOnce(() -> runShoulder(0));
  }
}
