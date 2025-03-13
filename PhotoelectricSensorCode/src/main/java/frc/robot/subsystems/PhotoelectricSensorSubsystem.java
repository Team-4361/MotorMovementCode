package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 


import java.util.Map;

//import static edu.wpi.first.math.filter.Debouncer.DebounceType.kBoth;
import static java.util.Map.entry;

public class PhotoelectricSensorSubsystem extends SubsystemBase {
    private final DigitalInput sensor1;
    private final DigitalInput sensor2;
    //private final Debouncer debouncer;
    //private boolean sensorActivated = false;

    public PhotoelectricSensorSubsystem(){
            this.sensor1 = new DigitalInput(Constants.PHOTOELECTRIC_SENSOR_1_PORT);
            this.sensor2 = new DigitalInput(Constants.PHOTOELECTRIC_SENSOR_2_PORT);


    }
    public boolean getSensor1()
    {
        return sensor1.get();
    }
    public boolean getSensor2()
    {
        return sensor2.get();
    }
    @Override
    public void periodic()
    {
        //SmartDashboard.putString("Sensor 1 value", "" + sensor1.get());
        //SmartDashboard.putString("Sensor 2 value", "" + sensor2.get());
    }
}