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

public class SensorBasedSubsystem extends SubsystemBase {
    //declare physical compents, private so no other folder can access it 
    private SparkMax motor; // This code is the logic for a the REV motors controlled by the sparkmaxes, i.e. 550 or neo 
    private final DigitalInput sensor1; //photoelectric sensors typically, however anything plugged into the DIO port works 
    private final DigitalInput sensor2;
    private final RelativeEncoder encoder;
    public boolean activated; // to record if a certain condition is triggered by the sensors

    //all final variables, the ones that stay constant throughout the code    
    private static final int MOTOR_ID = 1; // SparkMax ID to make motor object,  
    private static final int SENSOR_1_PORT = 0; //DIO port for first sensor, typically in constant folder 
    private static final int SENSOR_2_PORT = 1; //DIO port for second sensor, typically in constant folder
    public final boolean isDebug = false; //used to toggle data values when testing, usually in a constant folder


    public SensorBasedSubsystem() {
        //initialize variables
        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless); //550s, neos, and Krakens are all brushless, but you have to still declare if the motor is brushless 
        this.sensor1 = new DigitalInput(SENSOR_1_PORT); // sensor 1
        this.sensor2 = new DigitalInput(SENSOR_2_PORT); //  sensor 2 
        activated = false;  
              
        encoder = motor.getEncoder();
        SparkMaxConfig config = new SparkMaxConfig(); //making a configuration object for the motor's specs
        config.idleMode(IdleMode.kBrake); //sets the control of the motor in the brake mode
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //sets the 
    }

    // a method that is ran every 20 ms, code should be minimal and for data updates 
    @Override
    public void periodic() {
        //Puts sensor data onto driver station
        SmartDashboard.putBoolean("Sensor 1 Condition", getSensor1());
        SmartDashboard.putBoolean("Sensor 2 Condition", getSensor2());
       if (isDebug) //if the debug boolean is true, the needed values are also displayed, turn off during comp.
        {
            SmartDashboard.putString("Sensor 1 value", "" + sensor1.get());
            SmartDashboard.putString("Sensor 2 value", "" + sensor2.get());
        }
        if (sensor2.get() == false) 
        {
             activated = true;
        } 
    }
    public void stop()
    {
        motor.stopMotor();
    }
    public void ResetEncoder()
    {
        encoder.setPosition(0.0);
    }

    public double GetEncoderPos()
    {
        return encoder.getPosition();
    }
    public void SetMotorSpeed(Double speed) {
        //sets the speed of the motor
        motor.set(speed);
    }

    public boolean getSensor1()
    {
        return sensor1.get(); //Returns the first sensor's photon electric port
    }
    public boolean getSensor2()
    {
        return sensor2.get(); //Return the second sensor's phonton electric port
    }
    public void release()
    {
        //invert depending on rotation 
        motor.set(0.3); 
    }
    public void setMotor()
    {
        //sensors are normally true, when they change to false, the action is triggered
        if(!getSensor2())
        {
            motor.stopMotor();
        }
        else if(!getSensor1())
        {
            motor.set(0.70);
        } 
    }
}
