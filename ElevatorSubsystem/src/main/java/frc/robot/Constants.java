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
  public static final boolean isManual = true;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Coral
  { public static final int CPR_TALON = 2048;
    public static final double MOTOR_GEAR_RATIO = 1.0;
    public static final int LEFT_ELEVATOR_ID = 9;
    public static final int RIGHT_ELEVATOR_ID = 14;
    public static final int BUCKET_ID = 15;
    public static final double KP = 0.0666;
    public static final double KI = 0.00002;
    public static final double KD = 0.0010; 
    public static double L1_POS;
    public static double L2_POS;
    public static double L3_POS;
    public static double L4_POS;
    public static final double ELEVATOR_SPEED = 0.15;
    
  }
}
