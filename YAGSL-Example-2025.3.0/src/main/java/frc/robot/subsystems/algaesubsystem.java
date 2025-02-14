package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class algaesubsystem extends SubsystemBase {
    private SparkMax sparkMax;
    private RelativeEncoder encoder;
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private static final double POSITION_TOLERANCE = 0.02;

    // PID constants
    private static final double kP = 1.0;
    private static final double kI = 0.2;
    private static final double kD = 0.1;

    private double integral = 0.0;
    private double previousError = 0.0;
    private double targetPosition = 0.0; // Target position for PID control

    public algaesubsystem() {
        leftMotor = new SparkMax(Constants.Algae.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Algae.RIGHT_MOTOR_ID, MotorType.kBrushless);
        sparkMax = new SparkMax(Constants.Algae.ALGAE_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(0.0143);
        config.idleMode(IdleMode.kBrake);

        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = sparkMax.getEncoder(); // Get encoder from SparkMax
    }

    /** Sets the target position for the PID loop */
    public void setTargetPosition(double position) {
        targetPosition = position;
        integral = 0.0; // Reset integral term when setting a new target
        previousError = 0.0; // Reset previous error
    }

    /** Runs the PID loop to move the motor to the target position */
    @Override
    public void periodic() {
        double currentPosition = encoder.getPosition();
        double error = targetPosition - currentPosition;

        integral += error * 0.02; // Assuming a 20ms loop time
        double derivative = (error - previousError) / 0.02;

        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput)); // Limit output

        if (Math.abs(error) > POSITION_TOLERANCE) {
            sparkMax.set(pidOutput);
        } else {
            sparkMax.set(0);
        }

        previousError = error;

        // Debugging output
        System.out.println("Current Position: " + currentPosition);
        System.out.println("Target Position: " + targetPosition);
        System.out.println("PID Output: " + pidOutput);
    }

    /** Moves algae out (extrudes) */
    public void extrude() {
        leftMotor.set(-Constants.Algae.ALGAE_SPEED); 
        rightMotor.set(-Constants.Algae.ALGAE_SPEED);
    }

    /** Pulls algae in (sucks) */
    public void suck() {
        leftMotor.set(Constants.Algae.ALGAE_SPEED);
        rightMotor.set(Constants.Algae.ALGAE_SPEED);
    }

    /** Stops all motors */
    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
        sparkMax.set(0);
    }

}
