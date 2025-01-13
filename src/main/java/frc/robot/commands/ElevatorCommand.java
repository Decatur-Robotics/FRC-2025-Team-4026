package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    
    private ElevatorSubsystem elevatorSubsystem;
    private double targetPosition;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = targetPosition;
        this.addRequirements(this.elevatorSubsystem);
    }
    // use encoders for sensors//

    @Override
    public void initialize() {

        elevatorSubsystem.setTargetPosition(this.targetPosition);

    }
    @Override
    public void end(boolean interrupted) {

        elevatorSubsystem.setTargetPosition(this.targetPosition);

    }
  
    @Override
    public boolean isFinished() {
      //if poisiton is reached, isFinished should return false//
        if (this.targetPosition != elevatorSubsystem.getMotorPosition()) {
            
            return false;
        }
        return true;
        
    }
}
