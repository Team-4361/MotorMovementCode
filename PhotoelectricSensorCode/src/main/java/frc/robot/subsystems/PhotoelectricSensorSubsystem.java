package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import frc.robot.Constants; 


import java.util.Map;

//import static edu.wpi.first.math.filter.Debouncer.DebounceType.kBoth;
import static java.util.Map.entry;

public class PhotoelectricSensorSubsystem extends SubsystemBase {
    private final DigitalInput sensor1;
    private final DigitalInput sensor2;
    private final SparkBase SparkMax;
    //private final Debouncer debouncer;
    //private boolean sensorActivated = false;

    public PhotoelectricSensorSubsystem(){
            this.sensor1 = new DigitalInput(Constants.PHOTOELECTRIC_SENSOR_1_PORT);
            this.sensor2 = new DigitalInput(Constants.PHOTOELECTRIC_SENSOR_2_PORT);
            SparkMax = new SparkMax(6, MotorType.kBrushless);
    }
    public boolean getSensor1()
    {
        return sensor1.get();
    }
    public boolean getSensor2()
    {
        return sensor2.get();
    }
    public void stop()
    {
        SparkMax.set(0.0);
    }
    public void setMotor()
    {
        //SparkMax.set(0.1);
        if(!getSensor2())
        {
            SparkMax.stopMotor();
        }
        else if(!getSensor1())
        {
            SparkMax.set(0.10);
        }

        
    }


    @Override
    public void periodic()
    {
        SmartDashboard.putString("Sensor 1 value", "" + sensor1.get());
        SmartDashboard.putString("Sensor 2 value", "" + sensor2.get());
    }
}