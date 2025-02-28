package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
public class BucketMoveF45 extends Command {
private final BucketSubsystem coral;

    public BucketMoveF45(BucketSubsystem subsystem) {
        this.coral = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(coral);
    }

    @Override
    public void initialize()
    {
        if (!coral.atTarget());
            coral.forwardBucketAngle();
        //coral.forwardBucket();
    }
    @Override
    public void execute()
    {
        if (!coral.atTarget())
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
        return false; //idr how to implement this 
    }

}
