package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;


public class ZeroGyroCommand extends InstantCommand 
{
	private final SwerveSubsystem Swerve;

	public ZeroGyroCommand(final SwerveSubsystem Swerve)
	{
		this.Swerve = Swerve;
	}

	@Override
	public void initialize() 
	{
		Swerve.zeroGyro();
	}
}
