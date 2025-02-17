package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class WSTalon extends TimedRobot {
    private WPI_TalonSRX motor; // First Talon SRX motor controller
    private Encoder encoder;   // First encoder
    //private WPI_TalonSRX motor2; // Second Talon SRX motor controller
    ///private Encoder encoder2;   // Second encoder
    private PIDController pidController1; // PID for motor 1
    //private PIDController pidController2; // PID for motor 2
    private Joystick joystick; // Joystick for button control

    private static final int CPR = 2048; // Encoder counts per revolution
    private static final double MOTOR_GEAR_RATIO = 1.0;
    private static final double KP = 0.0666;
    private static final double KI = 0.00002;
    private static final double KD = 0.0010;

    private double targetAngle1 = 0.0; // Target angle for motor 1
    private double targetAngle2 = 0.0; // Target angle for motor 2

    @Override
    public void robotInit() {
        // Initialize motors and encoders
        motor = new WPI_TalonSRX(2);
        encoder = new Encoder(0, 1);
        //motor2 = new WPI_TalonSRX(12);
        //encoder2 = new Encoder(2, 3);
        joystick = new Joystick(0);

        encoder.setDistancePerPulse(360.0 / (CPR * MOTOR_GEAR_RATIO)); 
        //encoder2.setDistancePerPulse(360.0 / (CPR * MOTOR_GEAR_RATIO));

        // Set up PID controllers
        pidController1 = new PIDController(KP, KI, KD);
        //pidController2 = new PIDController(KP, KI, KD);
        pidController1.setTolerance(0.5);
        //pidController2.setTolerance(0.5);
    }

    @Override
    public void teleopPeriodic() {
        double joystickInput = joystick.getRawAxis(4); 
    
        double deadband = 0.05;
        if (Math.abs(joystickInput) < deadband) {
            joystickInput = 0;
        }
    
        double precisionScaling = 5.0;
        //targetAngle1 += joystickInput * precisionScaling;
        //targetAngle2 += joystickInput * precisionScaling;

        // Button-based control for motor 1
        if (joystick.getRawButtonPressed(1)) {
             targetAngle1 += 60.0; 
             targetAngle2 -= 60.0;
            
            } 
        if (joystick.getRawButtonPressed(3)) {
             targetAngle1 -= 60.0;
             targetAngle2 += 60.0; 
            } 
        if (joystick.getRawButtonPressed(2)) { targetAngle1 = 0.0; targetAngle2 = 0.0; } 
        if (joystick.getRawButtonPressed(4)) {
             encoder.reset();
             targetAngle1 = 0.0;
             //encoder2.reset(); 
             //targetAngle2 = 0.0;

            } 

        // Button-based control for motor 2

        if (joystick.getRawButtonPressed(7)) { targetAngle2 = 0.0; } 

        double currentAngle1 = encoder.getDistance();
        //double currentAngle2 = encoder2.getDistance();

        double pidOutput1 = pidController1.calculate(currentAngle1, targetAngle1);
        //double pidOutput2 = pidController2.calculate(currentAngle2, targetAngle2);

        pidOutput1 = Math.max(-1, Math.min(1, pidOutput1));
        //pidOutput2 = Math.max(-1, Math.min(1, pidOutput2));

        if (!pidController1.atSetpoint()) {
            motor.set(TalonSRXControlMode.PercentOutput, pidOutput1);
        } else {
            motor.set(TalonSRXControlMode.PercentOutput, 0);
        }

        /*if (!pidController2.atSetpoint()) {
            motor2.set(TalonSRXControlMode.PercentOutput, pidOutput2);
        } else {
            motor2.set(TalonSRXControlMode.PercentOutput, 0);
        }*/

         //System.out.println("Motor 2 -> Current Angle: " + currentAngle2 + " | Target: " + targetAngle2 + " | Output: " + pidOutput2);
    }
}
