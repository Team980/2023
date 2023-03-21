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

public class RollerGrabber extends PIDSubsystem {
    private ArmSensors sensors;

  private final double KS = .05;
  private final double GEAR_RATIO = 100;
  private final double POSITION_TOLERANCE = 2;

  private WPI_TalonSRX wrist; 
  private boolean isReversed;

  /** Creates a new RollerGrabber. */
  public RollerGrabber(ArmSensors sensors) {
    super(
        // The PIDController used by the subsystem
        new PIDController(12 / 100, 0, 0));

         wrist = new WPI_TalonSRX(9); // 8
        this.sensors = sensors;
        super.getController().setTolerance(POSITION_TOLERANCE);
        wrist.setNeutralMode(NeutralMode.Brake);
        wrist.setInverted(true);
        isReversed = false;

  }

  public void runWrist(double speed) {
    wrist.set(speed);
  }

  @Override
  public void useOutput(double output, double setpoint) {
        if(sensors.getSCon() && sensors.getWCon()) {
      wrist.setVoltage(output + customFFCalc(setpoint));
    }

      SmartDashboard.putNumber("W_PIDOut", output);
      SmartDashboard.putNumber("W_FF", customFFCalc(setpoint));
  }

  

  @Override
  public double getMeasurement() {
    setSetpoint(-sensors.getShoulderAngle() + 20);
    // Return the process variable measurement here
    return sensors.getWristAngle();
  }

  public CommandBase reverseGrab(boolean reverse){
    isReversed = reverse;
    if(reverse){
      return this.run(() -> setSetpoint(-90));
    }
    else{
      return this.run(() -> setSetpoint(90));
    }
  }

  public CommandBase stayLevel(){
    if(isReversed){
      return this.run(() -> setSetpoint(-180 - sensors.getShoulderAngle()));
    }
    else{
      return this.run(() -> setSetpoint(-sensors.getShoulderAngle()));
    }
  }

  public double customFFCalc(double goalPosition) { //direction is either 1 or -1 depending on the sensor
    double qs = sensors.getShoulderAngle();
    // double qe = sensors.getElbowAngle();
    double qw = sensors.getWristAngle();

    double csew = Math.cos(Math.toRadians(qs /*+ qe*/ + qw));
    
    double gWrist = csew * W_SEGMENT_MASS * W_CG_FROM_JOINT;

    return KS * Math.signum(goalPosition - qw) + (12 * gWrist / (BAG_MOTOR_STALL_TORQUE * GEAR_RATIO));//-12 change to voltage and oppose gravity
  }

}
