package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import frc.robot.Constants.drivingConstants;
public class SuckCommand {
    private SparkMax sparkMax;
    private RelativeEncoder encoder;
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private XboxController xboxController;

    // SPARK MAX CAN ID and speed settings
    private static final int SPARK_MAX_CAN_ID = 11;
    private static final int LEFT_MOTOR_ID = 6;  // Set to your left motor's CAN ID
    private static final int RIGHT_MOTOR_ID = 29; // Set to your right motor's CAN ID


    private static final double POSITION_TOLERANCE = 0.01;

    // PID constants
    private static final double kP = 2.0; // Proportional gain
    private static final double kI = 0.1; // Integral gain
    private static final double kD = 0.01; // Derivative gain

    // PID variables
    private double integral = 0.0;
    private double previousError = 0.0;

    // Target position for the motor
    private double targetPosition = 0.0;

   // @Override
   // public void robotInit() {
        // Initialize the SPARK MAX motor controller
       


        // Initialize the Xbox controller


        // Set the position conversion factor
    //}

    //@Override
    //public void teleopPeriodic() {
    public void negRotation()
    {
        targetPosition -= 4; // Increment by 1 rotation
    }
        

        // Increment position with Y button
        if (xboxController.getYButtonPressed()) {
            
        }

        // Reset position with X button
        if (xboxController.getXButtonPressed()) {
            targetPosition = 0.0;
        }

        // Reset encoder position to 0 with B button
        if (xboxController.getBButtonPressed()) {
            encoder.setPosition(0);
            targetPosition = 0.0; // Sync target position
        }

        // Decrement position with A button
        if (xboxController.getAButtonPressed()) {
            targetPosition =+ 4; // Decrement by 1 rotation
        }

        // Decrement position counterclockwise with Left Bumper
        if (xboxController.getLeftBumperPressed()) {
            targetPosition = -6; // Decrement by 0.5 rotation
        }

        // Increment position clockwise with Right Bumper
        if (xboxController.getRightBumperPressed()) {
            targetPosition = 6; // Increment by 0.5 rotation
        }
        
        if (rightBumperHeld) { //change to button imports?
            leftMotor.set(-0.5);
            rightMotor.set(-0.5);
        } else if (leftBumperHeld) {
            leftMotor.set(0.5);
            rightMotor.set(0.5);
        } else {
            leftMotor.set(0);
            rightMotor.set(0);
        }

        // Get the current position and calculate error
        double currentPosition = encoder.getPosition();
        double error = targetPosition - currentPosition;

        // Calculate the integral and derivative
        integral += error * 0.02; // Assuming teleopPeriodic runs at ~50 Hz (20 ms loop time)
        double derivative = (error - previousError) / 0.02;

        // Calculate PID output
        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);

        // Limit the PID output to the motor speed range
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput)); // Limit between -1.0 and 1.0

        // Set the motor output
        if (Math.abs(error) > POSITION_TOLERANCE) {
            sparkMax.set(pidOutput);
        } else {
            sparkMax.set(0); // Stop the motor if within tolerance
        }

        // Update the previous error
        previousError = error;

        if(tuningEnabled)
        {// Print current and target positions for debugging
            SmartDashboard.putNumber("Current Position", currentPosition);
            //System.out.println("Current Position: " + currentPosition);
            //System.out.println("Target Position: " + targetPosition);
            SmartDashboard.putNumber("Target Position", targetPosition);
            SmartDashboard.putNumber("PID Output", pidOutput);
            //System.out.println("PID Output: " + pidOutput);
        }
    }
    public void stopMotor() {
        // Stop the motor when the robot is disabled
        sparkMax.stopMotor();
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

}

