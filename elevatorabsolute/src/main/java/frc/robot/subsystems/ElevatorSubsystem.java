// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax motor1;
  private final SparkMax motor2;
  private final AbsoluteEncoder encoder1;
  private final AbsoluteEncoder encoder2;

  private final PIDController pidController;
  private final SimpleMotorFeedforward feedforward;

  private static final double kP = 0.5;  // Tune these values
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.2;  // Static friction voltage
  private static final double kG = 0.5;  // Gravity compensation voltage
  private static final double kV = 1.5;  // Velocity coefficient
  private static final double kA = 0.05; // Acceleration coefficient

  private double setpoint = 0.0; // Target position in meters

  public ElevatorSubsystem() {
      motor1 = new SparkMax(6, MotorType.kBrushless);
      motor2 = new SparkMax(12, MotorType.kBrushless);



    

      // Get absolute encoders from each motor
      //encoder1 = motor1.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle);
      encoder1 = motor1.getAbsoluteEncoder();//SparkRelativeEncoder.Type.kDutyCycle);
      encoder2 = motor2.getAbsoluteEncoder(); //gets their absolute encoders
      //encoder2 = motor2.getAbsoluteEncoder(com.revrobotics.AbsoluteEncoder.Type.kDutyCycle);


      pidController = new PIDController(kP, kI, kD); //defines PID
      feedforward = new SimpleMotorFeedforward(kS, kG, kV, kA); //defines feedforward values
  }



  private double getAveragePosition() {
      return (encoder1.getPosition() + encoder2.getPosition()) / 2.0; //Gets the average between the two positions
  }

  private double getAverageVelocity() {
      return (encoder1.getVelocity() + encoder2.getVelocity()) / 2.0; //Gets the average between the two positions
  }
  public boolean atSetpoint(double setpoint)
  {
    return getAveragePosition() == setpoint; //Sets their averages as the setpoint
  }

  public void setElevatorPosition(double positionMeters) {
      setpoint = positionMeters; //the position's meters is where the two motors will go to
  }

  
  @Override
  public void periodic() {
      double position = getAveragePosition(); // Currently, 0.0 - 1.0 of rotation of the encoder, needs testing 
      double velocity = getAverageVelocity(); // Velocity in m/s

      double pidOutput = pidController.calculate(position, setpoint);
      double ffOutput = feedforward.calculate(setpoint - position, velocity);

      double output = pidOutput + ffOutput;

      motor1.set(output); //sets their output from the PID and FF values
      motor2.set(output);

      SmartDashboard.putNumber("Elevator Position", position); //puts the numbers for the position, set point, and outpoint
      SmartDashboard.putNumber("Elevator Target", setpoint);
      SmartDashboard.putNumber("Elevator Output", output);
  }

  public void stop() {
      motor1.stopMotor(); //stops the motors from moving
      motor2.stopMotor();
  }
}