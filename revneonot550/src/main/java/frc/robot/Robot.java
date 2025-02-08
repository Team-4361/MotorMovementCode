package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants;


public class Robot extends TimedRobot {
    // Declare motor, encoder, and controller
    private SparkMax sparkMax;
    private SparkMax lElevatorSparkMax;
    private SparkMax rElevatorSparkMax;
    public static double elevatorSpeed = 0.5;
    public static int leftElevatorMotorID = 9;
    public static int rightElevatorMotorID = 10;
    public static PIDController elevatorPID = new PIDController(0.0,0.0, 0.0);
    private RelativeEncoder eEncoderL;
    private RelativeEncoder eEncoderR;
    private RelativeEncoder encoder;
    private XboxController xboxController;
    private Joystick lJoystick;

    // SPARK MAX CAN ID and speed settings
    private static final int SPARK_MAX_CAN_ID = 6;
    private static final double POSITION_TOLERANCE = 0.02;

    // PID constants
    private static final double kP = 0.5; // Proportional gain
    private static final double kI = 0.2; // Integral gain
    private static final double kD = 0.0; // Derivative gain

    // PID variables
    private double integral = 0.0;
    private double previousError = 0.0;

    // Target position for the motor
    private double targetPosition = 0.0;
    private double elevatorPosition = 0.0;
    @Override
    public void robotInit() {
        // Initialize the SPARK MAX motor controller
        sparkMax = new SparkMax(SPARK_MAX_CAN_ID, MotorType.kBrushless);
        lElevatorSparkMax = new SparkMax(leftElevatorMotorID, MotorType.kBrushless);
        rElevatorSparkMax = new SparkMax(rightElevatorMotorID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        SparkMaxConfig eConfig = new SparkMaxConfig();
        config.encoder.positionConversionFactor(9.0 / 40.0); // Example: Adjust for gear ratio or scaling
        eConfig.encoder.positionConversionFactor(kDefaultPeriod);
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lElevatorSparkMax.configure(eConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rElevatorSparkMax.configure(eConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // Initialize the Xbox controller
        xboxController = new XboxController(0);

        // Retrieve the encoder
        encoder = sparkMax.getEncoder();
        eEncoderL = lElevatorSparkMax.getEncoder();
        eEncoderR = rElevatorSparkMax.getEncoder();
        // Set the position conversion factor (e.g., 1 motor rotation = 5 inches)
        double inchesPerRotation = 5.0;

        // Set the position conversion factor
    }

    @Override
    public void teleopPeriodic() {
        if(lJoystick.getRawButtonPressed(4))
        {
            elevatorPosition = 30.6;
        }
        if(lJoystick.getRawButtonPressed(5))
        {
            elevatorPosition = 0;
        }
        

        // Increment position with Y button
        if (xboxController.getYButtonPressed()) {
            targetPosition = 3; // Increment by 1 rotation
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
            targetPosition = -3; // Decrement by 1 rotation
        }

        // Decrement position counterclockwise with Left Bumper
        if (xboxController.getLeftBumperPressed()) {
            targetPosition = -6; // Decrement by 0.5 rotation
        }

        // Increment position clockwise with Right Bumper
        if (xboxController.getRightBumperPressed()) {
            targetPosition = 6; // Increment by 0.5 rotation
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

        // Print current and target positions for debugging
        System.out.println("Current Position: " + currentPosition);
        System.out.println("Target Position: " + targetPosition);
        System.out.println("PID Output: " + pidOutput);
    }

    @Override
    public void disabledInit() {
        // Stop the motor when the robot is disabled
        sparkMax.stopMotor();
    }
}
