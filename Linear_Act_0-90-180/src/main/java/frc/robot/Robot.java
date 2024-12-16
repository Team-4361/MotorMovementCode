// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    // Declare motor, controller, and encoder
    private CANSparkMax sparkMax;

    private RelativeEncoder encoder;

    // Set CAN ID for the SPARK MAX
    private static final int SPARK_MAX_CAN_ID = 6;

    // PID coefficients
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kIz = 0.0;
    private static final double kFF = 0.0;
    private static final double kMaxOutput = 1.0;
    private static final double kMinOutput = -1.0;

    // Target position for the motor (in encoder units)
    private double targetPosition = 0.0;

    // Xbox controller for input
    private XboxController xboxController;

    @Override
    public void robotInit() {
        // Initialize motor
        sparkMax = new CANSparkMax(SPARK_MAX_CAN_ID, MotorType.kBrushless);

        // Restore factory defaults to ensure consistent behavior
        sparkMax.restoreFactoryDefaults();

        // Get the PID controller and encoder
        //pidController = sparkMax.getPIDController();
        encoder = sparkMax.getEncoder();

        // Configure PID coefficients
        //pidController.setP(kP);
        //pidController.setI(kI);
        //pidController.setD(kD);
        //pidController.setIZone(kIz);
        //pidController.setFF(kFF);
        //pidController.setOutputRange(kMinOutput, kMaxOutput);

        // Initialize Xbox controller (USB port 0)
        xboxController = new XboxController(0);
    }

    @Override
    public void teleopPeriodic() {
        // Check if Y button is pressed to increment position
        if (xboxController.getYButtonPressed()) {
            targetPosition += 36.5; // Increment target position
            while (encoder.getPosition()<targetPosition)
              sparkMax.set(0.1);
            sparkMax.set(0);
            
        }
        // set motor position to 0
        if (xboxController.getXButtonPressed()) {
            targetPosition = 0.0; // Increment target position
            while (encoder.getPosition()<targetPosition)
              sparkMax.set(0.1);
            sparkMax.set(0);
            
            
        }
        // This is to reset encoder position to 0
        if (xboxController.getBButtonPressed()) {
          
            sparkMax.getEncoder().setPosition(0);
            
        }
        

        // Check if A button is pressed to decrement position
        if (xboxController.getAButtonPressed()) {
            targetPosition -= 36.5; // Decrement target position
              while (encoder.getPosition()>targetPosition)
              {
                sparkMax.set(-0.1);
              }
              sparkMax.set(0);
            /*while (encoder.getPosition()>targetPosition && encoder.getPosition()>1)
              sparkMax.set(-0.1);
            sparkMax.set(0); */
          
        }

        // Send target position to the PID controller
       // pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);

        // Print current and target positions for debugging

   

        System.out.println("Current Position: " + encoder.getPosition());
        System.out.println("Target Position: " + targetPosition);
    }

    @Override
    public void disabledInit() {
        // Stop the motor when disabled
        sparkMax.stopMotor();
    }
}

