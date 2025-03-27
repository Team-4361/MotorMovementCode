package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;

public class BucketSubsystem extends SubsystemBase {
    private SparkMax coral;

    private RelativeEncoder encoder;
    
    public final boolean HasCoral;
   // private RelativeEncoder bucketEncoder;
    //private PIDController bucketPID;

    
    //private double targetPosition = 0.0;
    //private static final double GEAR_RATIO = 500.0; // 500:1 gear ratio
    //private static final double COUNTS_PER_REV = 42.0; // 42 counts per revolution
    //private static final double DEGREES_PER_MOTOR_REV = 360.0; // 1 motor rev = 360 degrees

   // private static final double POSITION_CONVERSION_FACTOR = 1;

    public BucketSubsystem() {
        //Declares variables
        coral = new SparkMax(16, MotorType.kBrushless);

        HasCoral = false;        
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        coral.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = coral.getEncoder();
        //driverStationJoystick = new Joystick(0);
        //limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        //bucketPID = new PIDController(Constants.Coral.KP, Constants.Coral.KI, Constants.Coral.KD);        
    }


    @Override
    public void periodic() {



        
    }

    public void SetMotorSpeed(Double speed) {
        coral.set(speed);
    }

    public void ResetEncoder() {
        encoder.setPosition(0.0);
    }

    public double GetEncoderPos() {
        return encoder.getPosition();
    }


    public void stop()
    {
        coral.set(0.0); //stops the mechanism
    }
    public void release()
    {
        //invert depending on rotation 
        coral.set(0.3); 
    }
    public void setMotor()
    {

        
    }

}
