package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import frc.robot.subsystems.AlgaeSubsystem;

public class BucketMoveToPosition extends Command {
    private final AlgaeSubsystem bucket;
    private final double targetDegrees;

    public BucketMoveToPosition(AlgaeSubsystem bucket, double degrees) {
=======
import frc.robot.subsystems.BucketSubsystem;

public class BucketMoveToPosition extends Command {
    private final BucketSubsystem bucket;
    private final double targetDegrees;

    public BucketMoveToPosition(BucketSubsystem bucket, double degrees) {
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
        this.bucket = bucket;
        this.targetDegrees = degrees;
        addRequirements(bucket);
    }

    @Override
    public void initialize() {
<<<<<<< HEAD
        bucket.setPosition(targetDegrees); 
=======
        bucket.setPosition(targetDegrees);
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
    }

    @Override
    public void execute() {
<<<<<<< HEAD
        bucket.setPosition(targetDegrees); //goes to the target degrees
=======
        bucket.setPosition(targetDegrees);
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
    }

    @Override
    public boolean isFinished() {
        return bucket.atSetpoint(); // Check if it's at the target position
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
<<<<<<< HEAD
            bucket.stop(); //if interrupted, stops the bucket
=======
            bucket.stop();
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
        }
    }
}
