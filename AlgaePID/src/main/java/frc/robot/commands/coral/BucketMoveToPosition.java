package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class BucketMoveToPosition extends Command {
    private final AlgaeSubsystem bucket;
    private final double targetDegrees;

    public BucketMoveToPosition(AlgaeSubsystem bucket, double degrees) {
        this.bucket = bucket;
        this.targetDegrees = degrees;
        addRequirements(bucket);
    }

    @Override
    public void initialize() {
        bucket.setPosition(targetDegrees); 
    }

    @Override
    public void execute() {
        bucket.setPosition(targetDegrees); //goes to the target degrees
    }

    @Override
    public boolean isFinished() {
        return bucket.atSetpoint(); // Check if it's at the target position
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            bucket.stop(); //if interrupted, stops the bucket
        }
    }
}
