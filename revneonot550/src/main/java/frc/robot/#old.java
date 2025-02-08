/* 
package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.spark.config.EncoderConfig;

public class old extends TimedRobot {
    // Declare motor, encoder, and controller
    private SparkMax sparkMax;
    private RelativeEncoder encoder;
    private XboxController xboxController;

    // SPARK MAX CAN ID and speed settings
    private static final int SPARK_MAX_CAN_ID = 11;
    private static final double MOTOR_SPEED = 0.1;
    private static final double POSITION_TOLERANCE = 0.00001;

    


    // Target position for the motor
    private double targetPosition = 0.0;

    @Override
    public void robotInit() {
        // Initialize the SPARK MAX motor controller
        sparkMax = new SparkMax(SPARK_MAX_CAN_ID, MotorType.kBrushless);

        encoder = sparkMax.getEncoder();

        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(1.0); // Set position to 1 per rotation
        encoder.configure(encoderConfig);


        xboxController = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        // Increment position with Y button
        if (xboxController.getYButtonPressed()) {
            targetPosition += 35.72901935;
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
            targetPosition -= 35.7502;
        }

        // Decrement position counterclockwise with Left Bumper
        if (xboxController.getLeftBumperPressed()) {
            targetPosition -= 17.5995;
        }

        // Increment position clockwise with Right Bumper
        if (xboxController.getRightBumperPressed()) {
            targetPosition += 17.7865;
        }

        // Gradually move toward the target position
        double currentPosition = encoder.getPosition();
        double error = targetPosition - currentPosition;

        if (Math.abs(error) > POSITION_TOLERANCE) {
            double direction = Math.signum(error); // Get direction (+1 or -1)
            sparkMax.set(direction * MOTOR_SPEED);
        } else {
            sparkMax.set(0); // Stop the motor if within tolerance
        }

        // Print current and target positions for debugging
        System.out.println("Current Position: " + currentPosition);
        System.out.println("Target Position: " + targetPosition);
    }

    @Override
    public void disabledInit() {
        // Stop the motor when the robot is disabled
        sparkMax.stopMotor();
    }
}
*/