package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
    
    private ClimberSubsystem climber;
    private double position;

    public ClimberCommand(ClimberSubsystem climber, double position) {
        this.climber = climber;
        this.position = position;
        
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setPosition(position);
    }
}
