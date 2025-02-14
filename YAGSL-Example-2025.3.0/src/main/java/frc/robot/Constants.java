// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(30)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(15.1);
  public static final SparkMax ALGAE_LEFT = new SparkMax(9, MotorType.kBrushless);
  public static final SparkMax ALGAE_RIGHT = new SparkMax(10, MotorType.kBrushless);
  public static final SparkMax ROTATION_ALGAE_ID = new SparkMax(11, MotorType.kBrushless);
  public static class Coral
  { public static final int CPR_TALON = 2048;
    public static final double MOTOR_GEAR_RATIO = 1.0;
    public static final double KP = 0.0666;
    public static final double KI = 0.00002;
    public static final double KD = 0.0010; 
  }
  

  // Maximum speed of the robot in meters per second, used to limit acceleration.
    public static final class drivingConstants
    {         /** The Left Joystick ID (typically 0) */
      public static final int LEFT_STICK_ID = 0;
      /** The Right Joystick ID (typically 1) */
      public static final int  RIGHT_STICK_ID = 1;
      /** The Xbox Controller (typically 2) */
      public static final int XBOX_ID = 2; 

    }
//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }


  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class Algae {
    public static final int LEFT_MOTOR_ID = 6;  // Set to your left motor's CAN ID
    public static final int RIGHT_MOTOR_ID = 29; // Set to your right motor's CAN ID
    public static final double ALGAE_SPEED = 0.75; //motorspeed

  }

}
