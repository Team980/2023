// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double BAG_MOTOR_STALL_TORQUE = 3.5/ 12.0; //ft-lbs

  public static final double SHO_SEGMENT_MASS = 5.4; //lbs
  public static final double SHO_SEGMENT_LENGTH = 32 / 12.0; //ft
  public static final double SHO_CG_FROM_JOINT = 9 / 12.0; //ft

  public static final double EL_SEGMENT_MASS = 5.21; //lbs
  public static final double EL_SEGMENT_LENGTH = 22 / 12.0; //ft
  public static final double EL_CG_FROM_JOINT = 17 / 12.0; //ft

  public static final double W_SEGMENT_MASS = 0; //3.89lbs
  public static final double W_SEGMENT_LENGTH = 0; //15 / 12.0ft
  public static final double W_CG_FROM_JOINT = 0; //5 / 12.0ft

  public static final double ACCEL_G = -32.17; //ft/s2

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
