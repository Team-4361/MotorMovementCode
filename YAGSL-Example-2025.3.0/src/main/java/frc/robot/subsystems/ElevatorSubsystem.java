package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.fasterxml.jackson.databind.cfg.ContextAttributes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    //private static final double POSITION_TOLERANCE = 0.02;
    private PIDController pidController1;
    private ElevatorFeedforward m_feedforward;
    private static final double kP = 0.0666;
    private static final double kI = 0.00002;
    private static final double kD = 0.0010;
    //private final RelativeEncoder lEncoder  = leftMotor.getEncoder();
    //private final RelativeEncoder rEncoder = rightMotor.getEncoder();
    //stuff from old talon code that idk is needed
    /*private double integral = 0.0;
    private double previousError = 0.0;
    private double targetPosition = 0.0; // Target position for PID control
    */
    

    public ElevatorSubsystem() {
        leftMotor = new SparkMax(Constants.Coral.LEFT_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Coral.RIGHT_ELEVATOR_ID, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        //config.encoder.positionConversionFactor(0.0143);
        config.idleMode(IdleMode.kBrake);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidController1 = new PIDController(kP, kI, kD);
        pidController1.setTolerance(0.5);
        ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0, 0.0);

    }

    /** Sets the target position for the PID loop */
   /*  public void setTargetPosition(double position) {
        targetPosition = position;
        integral = 0.0; // Reset integral term when setting a new target
        previousError = 0.0; // Reset previous error
    }*/


    /** Runs the PID loop to move the motor to the target position */
    @Override
    public void periodic() {}
    /*public void seedElevatorMotorPosition()
    {
        //yagsl code 
    }*/
    public void elevatorMoveUp()
    {
        leftMotor.set(Constants.Coral.ELEVATOR_SPEED);
        rightMotor.set(Constants.Coral.ELEVATOR_SPEED);
    }
    public void elevatorMoveDown()
    {
        leftMotor.set(-Constants.Coral.ELEVATOR_SPEED);
        rightMotor.set(-Constants.Coral.ELEVATOR_SPEED);
    }
    public void stopElevator()
    {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }

}