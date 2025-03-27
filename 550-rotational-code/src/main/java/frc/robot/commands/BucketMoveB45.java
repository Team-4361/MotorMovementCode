package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BucketSubsystem;
public class BucketMoveB45 extends Command {
private final BucketSubsystem coral;
private double currentAngle;

    public BucketMoveB45(BucketSubsystem subsystem) {
        this.coral = subsystem;
        // Declare subsystem dependency so no other command can use it at the same time.
        addRequirements(coral);
    }

    @Override
    public void initialize()
    {
        currentAngle = coral.getCurrentAngle(); //gets the current angle of the bucket
        coral.bBucket();
        //coral.backwardsBucket();
    }
    @Override
    public void execute()
    {
        
        coral.bBucket(); //moves the backet backwards
        //coral.backwardsBucket();
    }

    @Override
    public void end(boolean interrupted)
    {
        coral.stopBucket(); //stops if interrupted
    }

    @Override
    public boolean isFinished()
    {
        return false; //checks to see if it reached its position
    }

}