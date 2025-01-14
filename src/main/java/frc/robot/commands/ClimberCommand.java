package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    
    private ClimberSubsystem climberSubsystem;
    private double targetPosition;

    public ClimberCommand(ClimberSubsystem climberSubsystem, double targetPosition) {

        this.climberSubsystem = climberSubsystem;
        this.targetPosition = targetPosition;
        
        addRequirements(climberSubsystem);

    }

    @Override
    public void initialize() {

        climberSubsystem.setTargetPosition(targetPosition);

    }
}
