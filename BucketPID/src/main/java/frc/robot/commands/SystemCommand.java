package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SensorBasedSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class SystemCommand extends Command {
    private final SensorBasedSubsystem motor;
    private PIDController pidController;


    private static final double kP = 0.04; // Proportional gain
    private static final double kI = 0.0; // Integral gain
    private static final double kD = 0.0; // Derivative gain
    double targetPosition;

  /**

   * @param subsystem The subsystem used by this command.
   */
  public SystemCommand(SensorBasedSubsystem subsystem) {
    motor = subsystem;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    
    motor.ResetEncoder();
    targetPosition = -7;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //Uses PID & Tolerence set to go to the position
      pidController = new PIDController(kP, kI, kD);
      double currentPos =motor.GetEncoderPos();
      pidController.setTolerance(0.2);
      targetPosition = -7;
      
      double pidOutput = pidController.calculate(currentPos, targetPosition);
      pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
      SmartDashboard.putNumber("PidOutput:", pidOutput); //Debugging stuff
      SmartDashboard.putNumber("Stuff: ",motor.GetEncoderPos());
      SmartDashboard.putNumber("TargetPos", targetPosition);

    motor.SetMotorSpeed(pidOutput); //Sets the speed depending on the PID's output
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    motor.stop();
    motor.ResetEncoder();
      targetPosition = 0;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return pidController.atSetpoint(); // Stop when PID reaches setpointreturn false;
  }
    
}
