package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/* 
public class BucketSubsystem extends SubsystemBase {
    private final SparkMax bucketMotor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;

    private static final double GEAR_RATIO = 500.0; // 500:1 gear ratio
    private static final double COUNTS_PER_REV = 42.0; // 42 counts per revolution
    private static final double DEGREES_PER_MOTOR_REV = 360.0; // 1 motor rev = 360 degrees

    private static final double POSITION_CONVERSION_FACTOR = 1 / (GEAR_RATIO * COUNTS_PER_REV);

    // PID Constants
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public BucketSubsystem(int motorID) {
        bucketMotor = new SparkMax(motorID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR); // Converts encoder readings to degrees
        bucketMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = bucketMotor.getEncoder();
        
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(1.0); // Tolerance in degrees
    }

    public void setPosition(double degrees) {
        double output = pidController.calculate(encoder.getPosition(), degrees);
        bucketMotor.set(output);
    }

    public double getPosition() {
        return encoder.getPosition(); // Already in degrees due to conversion factor
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public void stop() {
        bucketMotor.set(0);
    }
}
*/

<<<<<<< HEAD
public class AlgaeSubsystem extends SubsystemBase {
=======
public class BucketSubsystem extends SubsystemBase {
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
    private SparkMax bucketMotor;
    private RelativeEncoder bucketEncoder;
   // private Joystick driverStationJoystick;
   // private DigitalInput limitSwitch;
    private PIDController winchPID;

    private static final int BUCKET_MOTOR_ID = 6;
<<<<<<< HEAD
    private static final double BUCKET_SPEED = 0.8; //sets the speed
=======
    private static final double BUCKET_SPEED = 0.8;
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)

    private static final double kP = 0.01;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    
    private double targetPosition = 0.0;
    private static final double GEAR_RATIO = 500.0; // 500:1 gear ratio
    private static final double COUNTS_PER_REV = 42.0; // 42 counts per revolution
    private static final double DEGREES_PER_MOTOR_REV = 360.0; // 1 motor rev = 360 degrees

    private static final double POSITION_CONVERSION_FACTOR = 1;

<<<<<<< HEAD
    public AlgaeSubsystem() {
=======
    public BucketSubsystem() {
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
        bucketMotor = new SparkMax(6, MotorType.kBrushless);
        bucketEncoder = bucketMotor.getEncoder();
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR); // Converts encoder readings to degrees
        bucketMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        //driverStationJoystick = new Joystick(0);
        //limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        winchPID = new PIDController(kP, kI, kD);
        

        

        
    }


    @Override
    public void periodic() {


        double currentPos = bucketEncoder.getPosition();
<<<<<<< HEAD
        double pidOutput = winchPID.calculate(currentPos, targetPosition); //PID stuff
=======
        double pidOutput = winchPID.calculate(currentPos, targetPosition);
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        bucketMotor.set(pidOutput);

        System.out.println("Current Position: " + currentPos);
<<<<<<< HEAD
        System.out.println("Target Position: " + targetPosition); //debugging stuff
=======
        System.out.println("Target Position: " + targetPosition);
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
        System.out.println("PID Output: " + pidOutput);


    }


    public void resetEncoder() {
        bucketEncoder.setPosition(0);
        targetPosition = 0.0; // Sync target position

    }

    public void setPosition(double degrees) {
<<<<<<< HEAD
        targetPosition = degrees; //sets the position
=======
        targetPosition = degrees;
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
    }


    public void winchMoveUp() {
<<<<<<< HEAD
        targetPosition = 30.0; //moves the winch up
=======
        targetPosition = 30.0;
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
    }


    public void stopWinch() {
<<<<<<< HEAD
        bucketMotor.stopMotor(); //stops the winch
    }

    public void stop() {
        bucketMotor.set(0); //stops the bucket
    }

    public boolean atSetpoint() {
        return winchPID.atSetpoint(); //returns whether its at the point or not
=======
        bucketMotor.stopMotor();
    }

    public void stop() {
        bucketMotor.set(0);
    }

    public boolean atSetpoint() {
        return winchPID.atSetpoint();
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
    }

}