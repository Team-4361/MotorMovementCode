package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 public class KerklunkSubsystem extends SubsystemBase 
 {
    //HS-322HD Servo 
    private Servo servo;
    private static final int kerklunkPort = 0;


    public KerklunkSubsystem() {
        servo = new Servo(kerklunkPort);
    }

    public void setAngle(double targetAngle)
    {     
        servo.setAngle(targetAngle);
    }
    public double getAngle()
    {
        return servo.getAngle();
    }
    public void zeroAngle() {
        servo.setAngle(0.0);
      
    }
} 