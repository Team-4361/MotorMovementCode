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


public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax LeftMotor;
    private SparkMax RightMotor;
    private RelativeEncoder LeftEncoder;
   // private Joystick driverStationJoystick;
   // private DigitalInput limitSwitch;
    private PIDController MotorPID;

    
    private double targetPosition = 0.0;
    private static final double GEAR_RATIO = 500.0; // 500:1 gear ratio
    private static final double COUNTS_PER_REV = 42.0; // 42 counts per revolution
    private static final double DEGREES_PER_MOTOR_REV = 360.0; // 1 motor rev = 360 degrees

    private static final double POSITION_CONVERSION_FACTOR = 1;

    public ElevatorSubsystem() {
        LeftMotor = new SparkMax(9, MotorType.kBrushless);
        RightMotor = new SparkMax(14, MotorType.kBrushless);
        LeftEncoder = LeftMotor.getEncoder();
        
        SparkMaxConfig config = new SparkMaxConfig();
        //config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR); // Converts encoder readings to degrees


        config.idleMode(IdleMode.kBrake);
        LeftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //configures the motors with safe parameters
        RightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        //driverStationJoystick = new Joystick(0);
        //limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        MotorPID = new PIDController(0.1, 0, 0);
        MotorPID.setTolerance(0.5);
        

        

        
    }


    @Override
    public void periodic() {


        double currentPos = LeftEncoder.getPosition();
        double pidOutput = MotorPID.calculate(currentPos, targetPosition);
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        LeftMotor.set(pidOutput);
        RightMotor.set(-pidOutput);



        SmartDashboard.putNumber("Elevator Current Position: ", currentPos);
        SmartDashboard.putNumber("Elevator Target Position: ", targetPosition);
        SmartDashboard.putNumber("Elevator PID Output: ", pidOutput);



       


    }


    public void setPower(double speed)
    {
        LeftMotor.set(speed);
    }
    public void setPosition(double degrees) {
        targetPosition = degrees;
    }



    public void stopBucket() {
        LeftMotor.stopMotor();
        RightMotor.stopMotor();
    }

    public void stop() {
        LeftMotor.set(0);
        RightMotor.set(0);
    }

    public boolean atSetpoint() {
        return MotorPID.atSetpoint();
    }

}
