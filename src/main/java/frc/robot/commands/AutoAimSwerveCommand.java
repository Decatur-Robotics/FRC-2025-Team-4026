package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.core.ILogSource;
import frc.robot.subsystems.SwerveSubsystem;


/**
 * Rotates the chassis towards the speaker. Intended to work with PathPlanner paths. Will end once
 * the note is fired. See {@link ShooterConstants#SHOOT_TIME} for the time to wait after shooting.
 */
public class AutoAimSwerveCommand extends Command implements ILogSource
{

	private final SwerveSubsystem Swerve;
	private double angle;

	public AutoAimSwerveCommand(SwerveSubsystem swerve, double angle)
	{
		Swerve = swerve;
		this.angle = angle;
	}

	@Override
	public void initialize()
	{
		logInfo("Starting AutoAimSwerveCommand");
	}

	@Override
	public void execute()
	{
		System.out.println(angle);
		Swerve.drive(new Translation2d(), Swerve.getRotationalVelocityToAngle(angle) * 3.7 * 3.7, true, false);
	}

	@Override
	public void end(boolean interrupted)
	{
		logInfo("Ending AutoAimSwerveCommand");
		Swerve.drive(new Translation2d(), 0, true, false);
	}

	@Override
	public boolean isFinished()
	{
		return true;
	}
}