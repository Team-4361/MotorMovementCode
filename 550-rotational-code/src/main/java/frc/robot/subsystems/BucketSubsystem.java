package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


//import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.fasterxml.jackson.databind.cfg.ContextAttributes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
    */
    /* 
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
// setup for PID movement of a NEO550
private final SparkMax motor;  //create the sparkmax motor object
private final RelativeEncoder encoder; // create the relative encoder that is attached to the motor, could change to alt 
private final PIDController pidController; //create the pidcontroller object, needs 3 values

private static final int MOTOR_ID = 6; // the id of the sparkmax that the motor is attached  
private static final int CPR = 42; // Encoder counts per revolution, was 2048 with the talon lea 42 for neo550
private static final double KP = 0.0666; // the P value of the PID controller, porportional 
private static final double KI = 0.00002; // the I value of the PID controller, intergral
private static final double KD = 0.0010; // the D value of te PID controller, derivative
private static final double MAX_POWER = 0.2; // the max power that the pidcontroller can set the motor to
//private final DCMotor bGearbox;
//private final Encoder e;

private double targetAngle1 = 0.0; // the desired encoder units that the motor needs to move to

public BucketSubsystem() {
    motor = new SparkMax(MOTOR_ID, MotorType.kBrushless); // initialtize the the motor with the id and with the brushless type 
    encoder = motor.getEncoder(); // initialtize the encoder object by using the getEncoder() method to interface with the physical encoder
    //if gearbox is used
    /*e.setDistancePerPulse(CPR);
    bGearbox = new DCMotor(MOTOR_GEAR_RATIO, MAX_POWER, KP, KI, KD, MOTOR_ID);*/

    //encoder.setDistancePerPulse(360.0 / (CPR * MOTOR_GEAR_RATIO));
    pidController = new PIDController(KP, KI, KD); //initialtize the pid controller with the related P, I, and D values
    pidController.setTolerance(0.5); // the unit tolerance in which the pidcontroller will stop moving the motor to the setpoint
}

public void forwardBucketAngle()
{
    targetAngle1 += 50; //add 50 encoder units to the target angle variable
}

public boolean atTarget()
{
    //if the encoder is currently at the target angle 
    return encoder.getPosition()>=targetAngle1;

}

public void backwardsBucketAngle()
{
    targetAngle1 -= 50;
}

public void fBucket() {
    //targetAngle1 += 5.0;
    motor.set(MAX_POWER); //sets the power for going forward
}

public void bBucket() {
    //targetAngle1 -= 5.0;
    motor.set(-MAX_POWER); //sets the power for going backwards
}
/* 
public void resetBucket() {
    //encoder.reset();
    targetAngle1 = 0.0;
}

public void zeroBucket() {
    targetAngle1 = 0.0;
}
*/
public void stopBucket() {
    motor.stopMotor(); //stops the bucket
}
public double getCurrentAngle(){
    return encoder.getPosition(); //gets the position of the bucket
}
public double getTargetAngle(){
    return targetAngle1; //gets a target angle

}
@Override
public void periodic() {
    double currentAngle = encoder.getPosition(); //gets the position of the bucket
    double pidOutput = pidController.calculate(currentAngle, targetAngle1); //tells the bucket its PID value

    pidOutput = Math.max(-1, Math.min(1, pidOutput)); //PID stuff

    pidOutput *= MAX_POWER;

    if (!pidController.atSetpoint()) {
        motor.set(pidOutput);
    } else {
        motor.set(0);
    }
    SmartDashboard.putNumber("current angle", currentAngle);
    SmartDashboard.putNumber("target angle", targetAngle1);
    SmartDashboard.putNumber("pid output", pidOutput);
    //System.out.println(currentAngle); //prints the current angle and PID output for debugging
    //System.out.println(pidOutput);

}
}
