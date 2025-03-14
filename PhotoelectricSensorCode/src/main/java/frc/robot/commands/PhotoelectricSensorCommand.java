package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PhotoelectricSensorSubsystem;

public class PhotoelectricSensorCommand extends Command{
//private PhotoelectricSensorSubsystem sensorSubsystem;

private final PhotoelectricSensorSubsystem coral;
private boolean sensor1;
private boolean sensor2;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PhotoelectricSensorCommand(PhotoelectricSensorSubsystem subsystem) {
    coral = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    coral.setMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

        coral.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
