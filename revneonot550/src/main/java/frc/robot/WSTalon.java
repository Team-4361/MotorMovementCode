package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class WSTalon extends TimedRobot {
    private WPI_TalonSRX motor; // Talon SRX motor controller
    private Encoder encoder;   // Encoder connected to DIO ports
    private PIDController pidController;
    private Joystick joystick; // Joystick for button control

    private static final int CPR = 2048; // Change this to your encoder's counts per revolution
    private static final double MOTOR_GEAR_RATIO = 1.0; // Change if you have a gearbox
    private static final double KP = 0.0666; // Increase for a stronger response
    private static final double KI = 0.0000; // Small integral gain
    private static final double KD = 0.0010; // Derivative gain for smooth control
    

    private double targetAngle = 0.0; // Current target angle

    @Override
    public void robotInit() {
        // Initialize motor, encoder, and joystick
        motor = new WPI_TalonSRX(2); // Replace '1' with your Talon SRX CAN ID
        encoder = new Encoder(0, 1); // Replace '0' and '1' with your DIO ports
        joystick = new Joystick(0);  // Replace '0' with your joystick's USB port

        encoder.setDistancePerPulse(360.0 / (CPR * MOTOR_GEAR_RATIO)); // Convert counts to degrees

        // Set up PID controller
        pidController = new PIDController(KP, KI, KD);
        pidController.setTolerance(0.5); // Tolerance in degrees
    }

    @Override
    public void teleopPeriodic() {
        // Read joystick axis (right thumbstick X-axis)
        double joystickInput = joystick.getRawAxis(4); 
    
        // Apply deadband to prevent small drift
        double deadband = 0.05;
        if (Math.abs(joystickInput) < deadband) {
            joystickInput = 0;
        }
    
        // Scale joystick input for precision movement
        double precisionScaling = 5.0; // Adjust max angle change per second
        targetAngle += joystickInput * precisionScaling;
    
        // Handle button-based angle adjustments
        if (joystick.getRawButtonPressed(1)) { // A button
            targetAngle += 60.0; 
        }
        if (joystick.getRawButtonPressed(3)) { // X button
            targetAngle -= 60.0; 
        }
        if (joystick.getRawButtonPressed(2)) { // B button
            targetAngle = 0.0; 
        }
        if (joystick.getRawButtonPressed(4)) { // Y button
            encoder.reset();
            targetAngle = 0.0; 
        }
        if (joystick.getRawButtonPressed(5)) { // Left bumper
            targetAngle += 90.0; 
        }
        if (joystick.getRawButtonPressed(6)) { // Right bumper
            targetAngle -= 90.0; 
        }
    
        // Get the current angle from the encoder
        double currentAngle = encoder.getDistance();
    
        // Calculate the PID output
        double pidOutput = pidController.calculate(currentAngle, targetAngle);
    
        // Limit the PID output to [-1, 1] to prevent excessive power
        pidOutput = Math.max(-1, Math.min(1, pidOutput));
    
        // Corrective action: ensure motor keeps moving until within tolerance
        if (!pidController.atSetpoint()) {
            motor.set(TalonSRXControlMode.PercentOutput, pidOutput);
        } else {
            motor.set(TalonSRXControlMode.PercentOutput, 0); // Stop motor when target is reached
        }
    
        // Debugging information
        System.out.println("Current Angle: " + currentAngle);
        System.out.println("Target Angle: " + targetAngle);
        System.out.println("PID Output: " + pidOutput);
    
        // Safety: Adjust target angle dynamically if drift is detected
        if (Math.abs(currentAngle - targetAngle) > 5.0) { // If error > 5 degrees
            System.out.println("Correcting angle drift...");
        }
    }
    
}    
