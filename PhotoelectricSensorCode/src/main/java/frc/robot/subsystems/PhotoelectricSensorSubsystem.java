package frc.robot.subsystems;

//all needed imports for system
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants; 

public class PhotoelectricSensorSubsystem extends SubsystemBase {
    
    // all needed variables for subsystem
    private final DigitalInput sensor1;
    private final DigitalInput sensor2;
    private final SparkMax SparkMax;
    private final RelativeEncoder encoder;

    public PhotoelectricSensorSubsystem(){
            //add the sensor port to the constants
            this.sensor1 = new DigitalInput(Constants.PHOTOELECTRIC_SENSOR_1_PORT);
            this.sensor2 = new DigitalInput(Constants.PHOTOELECTRIC_SENSOR_2_PORT);
            //change id for used sparkmax
            SparkMax = new SparkMax(15, MotorType.kBrushless);
            encoder = SparkMax.getEncoder();
            SparkMaxConfig config = new SparkMaxConfig();
            //sets config to brake for no movement after the sensor is triggered 
            config.idleMode(IdleMode.kBrake);
            //saves the brake configuration
            SparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public double getEncoder()
    {
        return encoder.getPosition();
    }
    public void zeroEncoder()
    {
        encoder.setPosition(0.0);
    }
    public boolean getSensor1()
    {
        return sensor1.get(); //gets if sensor 1 detects something
    }
    public boolean getSensor2()
    {
        return sensor2.get(); //gets if sensor 1 detects something
    }
    public void stop()
    {
        double targetPos = encoder.getPosition() - 7.0;
        if(encoder.getPosition() <= targetPos)
        {
            SparkMax.set(0.0); //sets the speed if it reaches the target position
        }
        else if(encoder.getPosition() > targetPos)
        {
            SparkMax.set(-0.1); //goes back if it goes over the target position
        }
        
        targetPos-=7.0;
    }
    public void release()
    {
        //invert depending on rotation 
        SparkMax.set(-0.3); 
    }
    public void setMotor()
    {
        //sensors are normally true, when they change to false, the action is triggered
        if(!getSensor2())
        {
            zeroEncoder();
            stop();
        }
        else if(!getSensor1())
        {
            SparkMax.set(-0.30);
        }

        
    }


    @Override
    public void periodic()
    {
        //for testing purposes, to see if sensor needs to be adjusted via the screw on the back
        SmartDashboard.putString("Sensor 1 value", "" + sensor1.get());
        SmartDashboard.putString("Sensor 2 value", "" + sensor2.get());
        SmartDashboard.putNumber("Bucket Position", encoder.getPosition());
    }
}