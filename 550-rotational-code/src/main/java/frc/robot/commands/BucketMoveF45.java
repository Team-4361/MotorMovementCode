package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
public class BucketMoveF45 extends Command {
private final BucketSubsystem coral;
private double currentAngle;
private double targetAngle;


    public BucketMoveF45(BucketSubsystem subsystem) {
        this.coral = subsystem;

        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(coral);
    }

    @Override
    public void initialize()
    {
            currentAngle = coral.getCurrentAngle(); 
            coral.forwardBucketAngle();

        //coral.forwardBucket();
    }
    @Override
    public void execute()
    {
        coral.forwardBucketAngle();
        //coral.forwardBucket();
    }

    @Override
    public void end(boolean interrupted)
    {
        coral.stopBucket();
    }

    @Override
    public boolean isFinished()
    {
        return currentAngle == coral.getTargetAngle(); //idr how to implement this 
    }

}
