package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    
    private ElevatorSubsystem elevatorSubsystem;
    private double targetPosition;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetPosition) {

        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = targetPosition;
        
        addRequirements(elevatorSubsystem);

    }

    @Override
    public void initialize() {

        elevatorSubsystem.setTargetPosition(targetPosition);

    }
}
