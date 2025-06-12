package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class bucketmove extends Command {
    private final AlgaeSubsystem bucket;

    public bucketmove(AlgaeSubsystem bucket) {
        this.bucket = bucket;

        addRequirements(bucket);
    }

    @Override
    public void initialize() {
        bucket.resetEncoder();
    }

    @Override
    public void execute() {
        bucket.resetEncoder(); //resets the encoder
    }


    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            bucket.stop(); //if interrupted, stops the bucket
        }
    }
}
