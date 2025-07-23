package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    private SparkMax bucketMotor;
    private RelativeEncoder bucketEncoder;
    private PIDController pidController;

    private static final int BUCKET_MOTOR_ID = 6;

    private static final double kP = 0.1; //PID damping values
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    
    private double targetPosition = 0.0;
    private static final double GEAR_RATIO = 500.0; // 500:1 gear ratio
    private static final double COUNTS_PER_REV = 42.0; // 42 counts per revolution
    private static final double POSITION_CONVERSION_FACTOR = 1 / (GEAR_RATIO*COUNTS_PER_REV);

    public AlgaeSubsystem() {
        bucketMotor = new SparkMax(BUCKET_MOTOR_ID, MotorType.kBrushless); //initializes brushless motor ID
        bucketEncoder = bucketMotor.getEncoder(); //initializes motor encoder
        
        SparkMaxConfig config = new SparkMaxConfig();//creates & initializes 
        //config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR); // Converts encoder readings to degrees
        bucketMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(0.5);
    }


    @Override
    public void periodic() {


        double currentPos = bucketEncoder.getPosition();
        double pidOutput = pidController.calculate(currentPos, targetPosition); //PID stuff
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        bucketMotor.set(pidOutput);

        SmartDashboard.putNumber("Current Position: ", currentPos);
        SmartDashboard.putNumber("Target Position: ", targetPosition); //debugging stuff
        SmartDashboard.putNumber("PID Output: ", pidOutput);


    }


    public void resetEncoder() {
        bucketEncoder.setPosition(0);
        targetPosition = 0.0; // Sync target position

    }

    public void setPosition(double degrees) {
        targetPosition = degrees; //sets the position
    }


    public void stopWinch() {
        bucketMotor.stopMotor(); //stops the winch
    }

    public void stop() {
        bucketMotor.set(0); //stops the bucket
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint(); //returns whether its at the point or not
    }

}