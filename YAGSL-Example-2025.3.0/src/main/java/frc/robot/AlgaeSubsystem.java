package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase{
    // initialized xbox rayz
    xboxController = new XboxController(0);

    // Retrieve the encoder
    encoder = sparkMax.getEncoder();

    // Set the position conversion factor (e.g., 1 motor rotation = 5 inches)
    double inchesPerRotation = 5.0;
        leftMotor = new SparkMax(ALGAE_LEFT, MotorType.kBrushless);
        rightMotor = new SparkMax(ALGAE_RIGHT, MotorType.kBrushless);
        sparkMax = new SparkMax(SPARK_MAX_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(0.01428); // Example: Adjust for gear ratio or scaling
        config.idleMode(IdleMode.kBrake);
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
}
}
