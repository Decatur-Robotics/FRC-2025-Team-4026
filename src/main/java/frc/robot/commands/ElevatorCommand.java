package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    
    private ElevatorSubsystem elevator;
    private double position;

    public ElevatorCommand(ElevatorSubsystem elevator, double position) {
        this.elevator = elevator;
        this.position = position;
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(position);
    }

}
