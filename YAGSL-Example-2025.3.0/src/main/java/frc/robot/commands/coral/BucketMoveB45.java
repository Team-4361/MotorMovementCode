package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
public class BucketMoveB45 extends Command {
private final BucketSubsystem coral;

    public BucketMoveB45(BucketSubsystem subsystem) {
        this.coral = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(coral);
    }

    @Override
    public void initialize()
    {
        if (!coral.atTarget())
            coral.backwardsBucketAngle();
        //coral.backwardsBucket();
    }
    @Override
    public void execute()
    {
        if (!coral.atTarget())
            coral.backwardsBucketAngle();
        //coral.backwardsBucket();
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
