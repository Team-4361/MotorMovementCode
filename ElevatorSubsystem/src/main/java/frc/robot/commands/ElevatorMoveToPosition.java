package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveToPosition extends Command {
    private final ElevatorSubsystem elevator;
    private final double targetDegrees;

    public ElevatorMoveToPosition(ElevatorSubsystem elevator, double position) {
        this.elevator = elevator;
        this.targetDegrees = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(targetDegrees);
    }

    @Override
    public void execute() {
        elevator.setPosition(targetDegrees);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint(); // Check if it's at the target position
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.stop();
        }
    }
} 
