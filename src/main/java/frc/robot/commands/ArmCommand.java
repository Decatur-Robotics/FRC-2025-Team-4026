package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command
{
	public ArmSubsystem arm;
	public double position;

	public ArmCommand(ArmSubsystem arm, double position)
	{		
		this.position = position;
		this.arm = arm;

		addRequirements(arm);
	
	}

	@Override
	public void initialize(){
		arm.setPosition(position);
	}
}


