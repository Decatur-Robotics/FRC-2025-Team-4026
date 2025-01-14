package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command
{
	public ArmSubsystem arm;
	public double targetAngle;

	public ArmCommand(ArmSubsystem arm, double targetAngle)
	{		
		this.targetAngle = targetAngle;

		this.arm = arm;
		addRequirements(arm);
	
	}

	@Override
	public void initialize(){
		arm.setTargetRotation(targetAngle);
	}
}