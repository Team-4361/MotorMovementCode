package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.fasterxml.jackson.databind.cfg.ContextAttributes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class BucketSubsystem extends SubsystemBase {
/*    private WPI_TalonSRX bucketTalon;
    private Encoder encoder;
    //private static final double POSITION_TOLERANCE = 0.02;
    private PIDController pidController1;
    private static final double kP = 0.0666;
    private static final double kI = 0.00002;
    private static final double kD = 0.0010;
    private static final int CPR = 2048; // Encoder counts per revolution
    private static final double MOTOR_GEAR_RATIO = 1.0;
    private double targetAngle1 = 0.0; 
    //private final RelativeEncoder lEncoder  = leftMotor.getEncoder();
    //private final RelativeEncoder rEncoder = rightMotor.getEncoder();
    //stuff from old talon code that idk is needed
    /*private double integral = 0.0;
    private double previousError = 0.0;
    private double targetPosition = 0.0; // Target position for PID control
    

    public BucketSubsystem() {
        bucketTalon = new WPI_TalonSRX(Constants.Coral.BUCKET_ID);
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(0.0143);
        config.idleMode(IdleMode.kBrake);
        encoder = new Encoder(0, 1); // Get encoder from SparkMax
        encoder.setDistancePerPulse(360.0 / (CPR * MOTOR_GEAR_RATIO)); 
        pidController1 = new PIDController(kP, kI, kD);
        pidController1.setTolerance(0.5);
        //ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0, 0.0);

    }

    /** Sets the target position for the PID loop */
   /*  public void setTargetPosition(double position) {
        targetPosition = position;
        integral = 0.0; // Reset integral term when setting a new target
        previousError = 0.0; // Reset previous error
    }


    /** Runs the PID loop to move the motor to the target position */
  /*   @Override
    public void periodic() {
        if(Constants.isDebug)
        {
            XboxController xbox = new XboxController(Constants.drivingConstants.XBOX_ID);
            double joystickInput = xbox.getLeftY();
            double deadband = 0.05;
            if (Math.abs(joystickInput) < deadband) 
            {
                joystickInput = 0;
            }
    
        double precisionScaling = 5.0;
        targetAngle1 += joystickInput * precisionScaling;
        //targetAngle2 += joystickInput * precisionScaling;

        }
        double currentAngle1 = encoder.getDistance();
        //double currentAngle2 = encoder2.getDistance();

        double pidOutput1 = pidController1.calculate(currentAngle1, targetAngle1);
        //double pidOutput2 = pidController2.calculate(currentAngle2, targetAngle2);

        pidOutput1 = Math.max(-1, Math.min(1, pidOutput1));

        if (!pidController1.atSetpoint()) {
            bucketTalon.set(TalonSRXControlMode.PercentOutput, pidOutput1);
        } else {
            bucketTalon.set(TalonSRXControlMode.PercentOutput, 0);
        }
        if(Constants.isDebug)
        {
            System.out.println("Motor 1 -> Current Angle: " + currentAngle1 + " | Target: " + targetAngle1 + " | Output: " + pidOutput1);
        }
}
    /*public void seedElevatorMotorPosition()
    {
        //yagsl code 
    }*//* *
    public void forwardBucket()
    {
        targetAngle1 += 45.0;   
    } 

    public void backwardsBucket()
    {
        targetAngle1 -= 45.0;
    }
    public void resetBucket()
    {
        encoder.reset();
        targetAngle1 = 0.0;
    }
    public void zeroBucket()
    {
        targetAngle1 = 0.0;
    }
    public void stopBucket()
    {
        
    }
*/
private final SparkMax motor;
private final Encoder encoder;
private final PIDController pidController;

private static final int MOTOR_ID = 15; // Change if needed
private static final int CPR = 2048; // Encoder counts per revolution
private static final double MOTOR_GEAR_RATIO = 1.0;
private static final double KP = 0.0666;
private static final double KI = 0.00002;
private static final double KD = 0.0010;
private static final double MAX_POWER = 0.15;

private double targetAngle1 = 0.0; 

public BucketSubsystem() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushed);
    encoder = new Encoder(0, 1); // External encoder on RoboRIO

    encoder.setDistancePerPulse(360.0 / (CPR * MOTOR_GEAR_RATIO));
    pidController = new PIDController(KP, KI, KD);
    pidController.setTolerance(0.5);
    SmartDashboard.putNumber("Encoder Pos" , encoder.getDistance());
   
}

public void forwardBucket() {
    //targetAngle1 += 5.0;
    motor.set(MAX_POWER);
}

public void backwardsBucket() {
    //targetAngle1 -= 5.0;
    motor.set(-MAX_POWER);
}

public void resetBucket() {
    encoder.reset();
    targetAngle1 = 0.0;
}

public void zeroBucket() {
    targetAngle1 = 0.0;
}

public void stopBucket() {
    motor.stopMotor();
}

@Override
public void periodic() {
    double currentAngle = encoder.getDistance();
    double pidOutput = pidController.calculate(currentAngle, targetAngle1);

    pidOutput = Math.max(-1, Math.min(1, pidOutput));

    pidOutput *= MAX_POWER;

    if (!pidController.atSetpoint()) {
        motor.set(pidOutput);
    } else {
        motor.set(0);
    }
    System.out.println(currentAngle);
    System.out.println(pidOutput);


}

}