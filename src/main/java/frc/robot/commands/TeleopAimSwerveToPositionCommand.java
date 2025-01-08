package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.SwerveSubsystem;



/**
 * Rotates the chassis towards the a specified rotation but allows the driver to control translation
 * and strafe
 */
public class TeleopAimSwerveToPositionCommand extends TeleopSwerveCommand
{

	private final SwerveSubsystem Swerve;

	public TeleopAimSwerveToPositionCommand(SwerveSubsystem swerve,
			DoubleSupplier translationSup, DoubleSupplier strafeSup,
			BooleanSupplier slowSpeedSupplier, double desiredRotation)
	{
		super(swerve, translationSup, strafeSup,
				() -> swerve.getRotationalVelocityToAngle(desiredRotation), slowSpeedSupplier);

		Swerve = swerve;
	}

	@Override
	public void execute()
	{
		super.execute();
	}

	@Override
	public void end(boolean interrupted)
	{
		Swerve.setRotationController(null);
	}

}
