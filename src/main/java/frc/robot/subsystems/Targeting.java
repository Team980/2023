// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Targeting extends SubsystemBase {
  private final double HEIGHT_OF_TARGET = 2.3; //in feet
  private final double HEIGHT_OF_H_STATION = 3.3; //in feet
  private final double HEIGHT_OF_CAMERA = 37.5 / 12;//TODO get real measurements
  private final double MOUNT_ANGLE = 20; //TODO get real measurements
  private final double DIST_CAM_TO_BUMPER = 1.5;//in feet TODO need actual measurement
  private final double HIGH_SCORE_ARM_LENGTH = 3.73;//in feet from edge of bumper TODO need actual length
  private final double MID_SCORE_ARM_LENGTH = 2.5;//in feet from edge of bumper TODO need actual length
  private final double DIST_HIGH_TO_TAG = 2.125;//in feet
  private final double DIST_MID_TO_TAG = .71;//in feet




  private NetworkTable limelight;
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  private double validTarget;
  private double x;
  private double y;
  private double a;

  /** Creates a new Targeting. */
  public Targeting() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    tv = limelight.getEntry("tv");
    tx = limelight.getEntry("tx");
    ty = limelight.getEntry("ty");
    ta = limelight.getEntry("ta");

    validTarget = 0;
    x = 100;
    y = 100;
    a = -1;

    limelight.getEntry("ledMode").setNumber(1);
    limelight.getEntry("camMode").setNumber(0);
    limelight.getEntry("pipeline").setNumber(0);
  }

  public void changeTag(int tag){//1 rl, 2 rc, 3 rr, 5 br, 6 bl, 7 bc
    limelight.getEntry("pipeline").setNumber(tag);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    validTarget = tv.getDouble(0);
    x = tx.getDouble(0);
    y = ty.getDouble(0);
    a = ta.getDouble(0);

    SmartDashboard.putNumber("Limelight Valid Target", validTarget);
    SmartDashboard.putNumber("limelight x", x);
    SmartDashboard.putNumber("limelight y", y);
    //SmartDashboard.putNumber("limelight a", a);
    SmartDashboard.putNumber("range", calcRange());
  }

  public double getValidTarget(){
    return validTarget;
  }
  
  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getA() {
    return a;
  }

  public void ledOn(boolean on) {
    NetworkTableEntry led = limelight.getEntry("ledMode");
    if(on) {
      led.setNumber(3);
    } else {
      led.setNumber(1);
    }
  }

  public double calcRange() {
    double d = (HEIGHT_OF_TARGET - HEIGHT_OF_CAMERA) / Math.tan(Math.toRadians(MOUNT_ANGLE + y));
    return d;     
  }

  public double calcOffset(){
    double ao = (1.1875 + DIST_CAM_TO_BUMPER) * Math.tan(Math.toRadians(Math.abs(x)));// right is negative
    if(x > 0){
      ao *= -1;
    }
    return ao;
  }

  public double[] getRangeAngle(int target){//0 high cube, 1 mid cube, 2 high cone, 3 mid cone, array[0] desired is range from tag, array[1] is angle to target, will choose side based on which side of tag
    double[] output = {-1 , 90};//impossible values for error checking
    if (target == 0){
      output[0] = Math.sqrt((HIGH_SCORE_ARM_LENGTH * HIGH_SCORE_ARM_LENGTH) - (calcOffset() * calcOffset())) - DIST_HIGH_TO_TAG;
      output[1] = Math.signum(calcOffset()) * (Math.asin(Math.abs(calcOffset()) / HIGH_SCORE_ARM_LENGTH));
    }
    else if (target == 1){
      output[0] = Math.sqrt((MID_SCORE_ARM_LENGTH * MID_SCORE_ARM_LENGTH) - (calcOffset() * calcOffset())) - DIST_MID_TO_TAG;
      output[1] = Math.signum(calcOffset()) * (Math.asin(Math.abs(calcOffset()) / MID_SCORE_ARM_LENGTH));
    }
    else if (target == 1){
      double hRangeTarget = 1.5 - Math.abs(calcOffset());
      output[0] = Math.sqrt((HIGH_SCORE_ARM_LENGTH * HIGH_SCORE_ARM_LENGTH) - (hRangeTarget * hRangeTarget)) - DIST_HIGH_TO_TAG;
      output[1] = -1 * Math.signum(calcOffset()) * (Math.asin(Math.abs(calcOffset()) / HIGH_SCORE_ARM_LENGTH));
    }
    else if (target == 1){
      double hRangeTarget = 1.5 - Math.abs(calcOffset());
      output[0] = Math.sqrt((MID_SCORE_ARM_LENGTH * MID_SCORE_ARM_LENGTH) - (hRangeTarget * hRangeTarget)) - DIST_MID_TO_TAG;
      output[1] = -1 * Math.signum(calcOffset()) * (Math.asin(Math.abs(calcOffset()) / MID_SCORE_ARM_LENGTH));
    }

    return output;
  }

}