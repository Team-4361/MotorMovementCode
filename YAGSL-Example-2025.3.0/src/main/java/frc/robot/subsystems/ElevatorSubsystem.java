package frc.robot.subsystems;

import java.lang.annotation.Target;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.fasterxml.jackson.databind.cfg.ContextAttributes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    //private static final double POSITION_TOLERANCE = 0.02;
    private PIDController pidController1;
    private static final double kP = 0.0666;
    private static final double kI = 0.00002;
    private static final double kD = 0.0010;
    private final RelativeEncoder lEncoder;
    private final RelativeEncoder rEncoder;
    private final ElevatorFeedforward m_feedForward;
     private final ProfiledPIDController m_controller;
    //stuff from old talon code that idk is needed
    //private double integral = 0.0;
    //private double previousError = 0.0;
    private double targetPosition = 0.0; // Target position for PID control
    
    

    public ElevatorSubsystem() {
        leftMotor = new SparkMax(Constants.Coral.LEFT_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Coral.RIGHT_ELEVATOR_ID, MotorType.kBrushless);
        lEncoder  = leftMotor.getEncoder();
        rEncoder = rightMotor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig();
        m_controller = new ProfiledPIDController(
            Constants.ElevatorConstants.kElevatorKp,
            Constants.ElevatorConstants.kElevatorKi,
            Constants.ElevatorConstants.kElevatorKd,
            new TrapezoidProfile.Constraints(
                Constants.ElevatorConstants.kElevatorMaxVelocity,
                Constants.ElevatorConstants.kElevatorMaxAcceleration));
        m_feedForward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS,
            ElevatorConstants.kElevatorkG,
            ElevatorConstants.kElevatorkV,
            ElevatorConstants.kElevatorkA);
        //config.encoder.positionConversionFactor(0.0143);
        config.idleMode(IdleMode.kBrake);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidController1 = new PIDController(kP, kI, kD);
        pidController1.setTolerance(0.5);
    }
public double getPositionMeters() {
        return lEncoder.getPosition() * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public double getVelocityMetersPerSecond() {
        return (lEncoder.getVelocity() / 60) * (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius)
                / ElevatorConstants.kElevatorGearing;
    }

    public void reachGoal(double goal){
        double voltsOutput = MathUtil.clamp(
                m_feedForward.calculateWithVelocities(getVelocityMetersPerSecond(), m_controller.getSetpoint().velocity)
                + m_controller.calculate(getPositionMeters(), goal),
                -12,
                12);
        leftMotor.setVoltage(voltsOutput);
        rightMotor.setVoltage(-voltsOutput);

    }

    public Command setGoal(double goal){
        return run(() -> reachGoal(goal));
    }

    public Command setElevatorHeight(double height){
        return setGoal(height).until(()->aroundHeight(height));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionMeters(),tolerance);
    }

    /** Sets the target position for the PID loop */
   /* *public void setTargetPosition(double position) {
        targetPosition = position;
        //integral = 0.0; // Reset integral term when setting a new target
        //previousError = 0.0; // Reset previous error
    }

    public boolean atSetpoint() {
        return pidController1.atSetpoint();
    }

    public void setPosition(double height) {
        double output = pidController1.calculate(lEncoder.getPosition(), height);
        leftMotor.set(output);
        rightMotor.set(-output);
    } */


    /** Runs the PID loop to move the motor to the target position */
    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("Left Encoder Position", lEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder Position", rEncoder.getPosition());
        double pidOutput1 = pidController1.calculate(lEncoder.getPosition(), targetPosition);
        double pidOutput2 = pidController1.calculate(rEncoder.getPosition(), -targetPosition);



    }
    /*public void seedElevatorMotorPosition()
    {
        //yagsl code 
    }*/
    public void elevatorMoveUp()
    {
        if(lEncoder.getPosition() > 146.8 || rEncoder.getPosition() < -146.8)
        {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }
        else
        {
            leftMotor.set(Constants.Coral.ELEVATOR_SPEED);
            rightMotor.set(-Constants.Coral.ELEVATOR_SPEED);

        }

    }


    public void elevatorMoveDown()
    {
        if (lEncoder.getPosition() < 5 ||  rEncoder.getPosition() > -5 ) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }
        else {

            leftMotor.set(-Constants.Coral.ELEVATOR_SPEED);
            rightMotor.set(Constants.Coral.ELEVATOR_SPEED);

        }



    }
    public void stopElevator()
    {
        leftMotor.set(0.0);
        rightMotor.set(0.0);
    }


}