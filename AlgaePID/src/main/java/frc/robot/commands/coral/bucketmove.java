package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import frc.robot.subsystems.AlgaeSubsystem;

public class bucketmove extends Command {
    private final AlgaeSubsystem bucket;

    public bucketmove(AlgaeSubsystem bucket) {
=======
import frc.robot.subsystems.BucketSubsystem;

public class bucketmove extends Command {
    private final BucketSubsystem bucket;

    public bucketmove(BucketSubsystem bucket) {
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
        this.bucket = bucket;

        addRequirements(bucket);
    }

    @Override
    public void initialize() {
        bucket.resetEncoder();
    }

    @Override
    public void execute() {
<<<<<<< HEAD
        bucket.resetEncoder(); //resets the encoder
=======
        bucket.resetEncoder();
>>>>>>> parent of 6d86ae0 (Updated BucketPID with PID now and updated subsystem from actual code and stuff)
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
