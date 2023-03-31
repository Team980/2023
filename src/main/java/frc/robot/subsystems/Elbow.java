// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;

public class Elbow extends PIDSubsystem { 
  /** Creates a new Elbow. */

  private ArmSensors sensors;

  private final double KS = .1;
  private final double GEAR_RATIO = 237;
  private final double POSITION_TOLERANCE = 2;

  private double currentPosition;

  private WPI_TalonSRX elbow;

  public Elbow(ArmSensors sensors) {
    super(
        // The PIDController used by the subsystem
        new PIDController(12.0 / 180, 0, 0));

        elbow = new WPI_TalonSRX(9);
        elbow.setInverted(false);
        elbow.setNeutralMode(NeutralMode.Brake);

        this.sensors = sensors;
        currentPosition = getMeasurement();
        super.getController().setTolerance(POSITION_TOLERANCE);
       // enable();
        //setSetpoint(currentPosition);//147
  }

  @Override
  public void useOutput(double output, double setpoint) {  
    // Use the output here
    elbow.set(Math.signum(output) * .5);
    // if(sensors.getSCon() || sensors.getECon() || sensors.getWCon())
      //elbow.setVoltage(output + customFFCalc(setpoint)); 
  }

  public void runElbow(double speed) {
    //if(Math.abs(speed) > 0.1){
      /*if((sensors.getElbowAngle() >= 140 && speed > 0)) {
        //disable();
        elbow.stopMotor();
      }
      else {*/
      
        elbow.set(speed);
        //currentPosition = getMeasurement();
      //}
    //}
    //else{
      //enable();
      //setSetpoint(currentPosition);
    //}


  }

  @Override
  public double getMeasurement() {

    // Return the process variable measurement here
    return sensors.getElbowAngle();
  }

  public double customFFCalc(double goalPosition) { //direction is either 1 or -1 depending on the sensor
    double qs = sensors.getShoulderAngle();
    double qe = sensors.getElbowAngle();
    double qw = sensors.getWristAngle();

    double cse = Math.cos(Math.toRadians(qs + qe));
    double csew = Math.cos(Math.toRadians(qs + qe + qw));
    
    double gElbow = (EL_SEGMENT_MASS * EL_CG_FROM_JOINT + W_SEGMENT_MASS * EL_SEGMENT_LENGTH) * cse + 
      csew * W_SEGMENT_MASS * W_CG_FROM_JOINT;

    return KS * Math.signum(goalPosition - qe) + (12 * gElbow / (SEVEN_MOTOR_STALL_TORQUE * GEAR_RATIO));//-12 change to voltage and oppose gravity
  }

  public void kindaManualE(double moveF , double moveR) {
    if(Math.abs(moveF) > 0.2) {
      setSetpoint(moveF + getSetpoint());
    }
    else if(Math.abs(moveR) > 0.2){
      setSetpoint(moveR + getSetpoint());
    }
  }

  public CommandBase holdPosition(){
    return this.runOnce(() -> setSetpoint(getMeasurement()));
  }

}
