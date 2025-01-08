package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAimSwerveCommand;
import frc.robot.commands.TeleopAimSwerveToPositionCommand;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.constants.SwerveConstants;
import frc.robot.core.ILogSource;
import frc.robot.core.LogitechControllerButtons;


public class SwerveSubsystem extends SubsystemBase implements ILogSource
{

	private SwerveDriveOdometry swerveOdometry;
	private SwerveDrivePoseEstimator swervePoseEstimator;
	private SwerveModule[] swerveMods;
	private Pigeon2 gyro;

	private double gyroOffset = 0;

	private Optional<DoubleSupplier> rotationController;

	private ProfiledPIDController autoAimPidController;

	public SwerveSubsystem()
	{
		gyro = null;

		zeroGyro();

		setAngleOffsets(false);

		// swerve module initialization
		swerveMods = new SwerveModule[]
		{
				new SwerveModule(SwerveConstants.FRONT_LEFT, SwerveConstants.ModFL.Constants),
				new SwerveModule(SwerveConstants.FRONT_RIGHT, SwerveConstants.ModFR.constants),
				new SwerveModule(SwerveConstants.BACK_LEFT, SwerveConstants.ModBL.Constants),
				new SwerveModule(SwerveConstants.BACK_RIGHT, SwerveConstants.ModBR.Constants)
		};

		/*
		 * By pausing init for a second before setting module offsets, we avoid a bug with inverting
		 * motors. See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
		 */
		Timer.delay(1.0);
		resetModulesToAbsolute();

		// construct odometry (full robot position/incorporated module states)
		swerveOdometry = new SwerveDriveOdometry(SwerveConstants.SwerveKinematics, getYaw(),
				getModulePositions());

		swervePoseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.SwerveKinematics,
				getYaw(), getModulePositions(), swerveOdometry.getPoseMeters());

		

		rotationController = Optional.empty();

		autoAimPidController = new ProfiledPIDController(SwerveConstants.ANGULAR_AIMING_KP,
				SwerveConstants.ANGULAR_AIMING_KI, SwerveConstants.ANGULAR_AIMING_KD,
				SwerveConstants.ANGULAR_VELOCITY_CONSTRAINTS);

                try{
                    config = RobotConfig.fromGUISettings();
                  } catch (Exception e) {
                    // Handle exception as needed
                    e.printStackTrace();
                  }
                      // Type::method gets a reference to the method. Type.method only allows us to
                      // run the method
              
                      AutoBuilder.configure(this::getPose, this::resetPose, this::getCurrentSpeeds,
                              this::drive, (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                                      new PIDConstants(5.0, 0.0, 0.0), 
                                      new PIDConstants(5.0, 0.0, 0.0), config, () -> {var alliance = DriverStation.getAlliance();
                                          if (alliance.isPresent()) {
                                            return alliance.get() == DriverStation.Alliance.Red;
                                          }
                                          return false;
                                        },
                                        this));
                                      
                  
	}

	

	public void setAngleOffsets(boolean invert)
	{
		System.out.println("Setting angle offsets...");

		double[] offsets = SwerveConstants.ANGLE_OFFSETS;

		SwerveConstants.ModFL.angleOffset = Rotation2d
				.fromDegrees(offsets[SwerveConstants.FRONT_LEFT] - (invert ? 180 : 0));
		SwerveConstants.ModFR.angleOffset = Rotation2d
				.fromDegrees(offsets[SwerveConstants.FRONT_RIGHT] - (invert ? 180 : 0));
		SwerveConstants.ModBL.angleOffset = Rotation2d
				.fromDegrees(offsets[SwerveConstants.BACK_LEFT] - (invert ? 180 : 0));
		SwerveConstants.ModBR.angleOffset = Rotation2d
				.fromDegrees(offsets[SwerveConstants.BACK_RIGHT] - (invert ? 180 : 0));
	}

	/** main driving method. translation is change in every direction */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative,
			boolean isOpenLoop)
	{
		drive(fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
						rotation, getYaw())
				: new ChassisSpeeds(translation.getX(), translation.getY(), rotation), isOpenLoop);

	}

	private void drive(ChassisSpeeds speeds, boolean isOpenLoop)
	{
		SwerveModuleState[] swerveModuleStates = SwerveConstants.SwerveKinematics
				.toSwerveModuleStates(speeds);

		// lowers module speeds to max attainable speed (avoids going above topspeed)
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

		// Log module state
		for (int i = 0; i < swerveModuleStates.length; i++)
		{
			SwerveModuleState mod = swerveModuleStates[i];
			SmartDashboard.putNumber("Mod " + i + " Target Angle", mod.angle.getDegrees());
			SmartDashboard.putNumber("Mod " + i + " Target - CANCoder",
					mod.angle.getDegrees() - swerveMods[i].getCanCoder().getDegrees());
			SmartDashboard.putNumber("Mod " + i + " Target Speed", mod.speedMetersPerSecond);
		}

		// sets modules to desired state (angle, speed)
		for (SwerveModule mod : swerveMods)
		{
			mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
		}
	}

	private void drive(ChassisSpeeds speeds)
	{
		// Override rotation if a controller is present
		// This override is used by autonomous to override PathPlanner
		if (rotationController.isPresent())
		{
			double rotation = rotationController.get().getAsDouble();
			speeds.omegaRadiansPerSecond = rotation;
		}

		speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
				-speeds.omegaRadiansPerSecond);

		drive(speeds, false);
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates)
	{
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

		for (SwerveModule mod : swerveMods)
		{
			mod.setDesiredState(desiredStates[mod.moduleNumber], false);
		}
	}

	/** @return position of robot on the field (odometry) in meters */
	public Pose2d getPose()
	{
		return swervePoseEstimator.getEstimatedPosition();
	}

	/** resets odometry (position on field) */
	public void resetPose(Pose2d pose)
	{
		setGyro(pose.getRotation().getDegrees());

		swervePoseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
	}

	/** @return array of a modules' states (angle, speed) for each one */
	public SwerveModuleState[] getModuleStates()
	{
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : swerveMods)
		{
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	/** @return module positions(for each individual module) */
	public SwerveModulePosition[] getModulePositions()
	{
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : swerveMods)
		{
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public SwerveModule[] getSwerveMods()
	{
		return swerveMods;
	}

	public void setGyro(double degrees)
	{
		System.out.println("Setting gyro to " + degrees + "...");
		gyroOffset = 0;
		gyro.setYaw(degrees);
	}

	public void zeroGyro()
	{
		System.out.println("Zeroing gyro");

		gyro.setYaw(0); // Used to setYaw(0);
		gyroOffset = 0;
	}

	public void setGyroOffset(double offset)
	{
		gyroOffset = offset;
	}

	/** Returns angle around vertical axis */
	public Rotation2d getYaw()
	{
		return (SwerveConstants.INVERT_GYRO)
				? Rotation2d.fromDegrees(360 - gyro.getYaw().getValueAsDouble() + gyroOffset)
				: Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble() + gyroOffset);
	}

	public void resetModulesToAbsolute()
	{
		for (SwerveModule mod : swerveMods)
		{
			mod.resetToAbsolute();
		}
	}

	@Override
	public void periodic()
	{
		// System.out.println(getPose().getX() + " " + getPose().getY());

		swerveOdometry.update(getYaw(), getModulePositions());
		swervePoseEstimator.update(getYaw(), getModulePositions());

		// smartdashboard logging per module
		for (SwerveModule mod : swerveMods)
		{
			mod.periodic();

			// SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
			// mod.getCanCoder().getDegrees());
			// SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
			// mod.getPosition().angle.getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
					mod.mDriveMotor.getVelocity().getValueAsDouble());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Current",
					mod.mDriveMotor.getSupplyCurrent().getValueAsDouble());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Voltage",
					mod.mDriveMotor.getSupplyVoltage().getValueAsDouble());
		}

		SmartDashboard.putNumber("Gyro Angle", getYaw().getDegrees());
	}

	public void resetEncoders()
	{
		for (SwerveModule mod : swerveMods)
		{
			mod.resetToAbsolute();
		}
	}

	public ChassisSpeeds getCurrentSpeeds()
	{
		return SwerveConstants.SwerveKinematics.toChassisSpeeds(getModuleStates());
	}

	public void setRotationController(final DoubleSupplier RotationController)
	{
		try
		{
			rotationController = Optional.of(RotationController);
		}
		catch (NullPointerException e)
		{
			rotationController = Optional.empty();
		}
	}

	/**
	 * @param Controller that controls the swerve drive
	 * @return the default command for the swerve drive that allows full driver control
	 */
	public TeleopSwerveCommand getDefaultCommand(final Joystick Controller)
	{
		return getTeleopControlledRotationCommand(Controller, () -> Math.pow(
				MathUtil.applyDeadband(Controller.getTwist(), SwerveConstants.JOYSTICK_DEADBAND),
				3));
	}

	public AutoAimSwerveCommand getAutoAimSwerveCommand(double angle)
	{
		double newAngle = DriverStation.getAlliance().get() == Alliance.Blue ? angle : 3.14 - angle;

		return new AutoAimSwerveCommand(this, newAngle);
	}

	/**
	 * @param Controller that controls the swerve drive
	 * @return a command for the swerve drive that allow driver control of translation and strafe,
	 *         but rotates towards the speaker and automatically spins the feeder motors when in
	 *         target
	 */
	public TeleopSwerveCommand getTeleopAimCommand(final Joystick Controller)
	{
		final JoystickButton BumperRight = new JoystickButton(Controller,
				LogitechControllerButtons.bumperRight);

		return new TeleopAimSwerveCommand();
	}

	public TeleopSwerveCommand getTeleopAimToPositionAllianceRelativeCommand(
			final Joystick Controller, double desiredRotation)
	{
		final JoystickButton BumperRight = new JoystickButton(Controller,
				LogitechControllerButtons.bumperRight);

		desiredRotation = DriverStation.getAlliance().get() == Alliance.Blue ? desiredRotation
				: Math.PI - desiredRotation;

		return new TeleopAimSwerveToPositionCommand(this, () -> -Controller.getY(),
				() -> -Controller.getX(), BumperRight::getAsBoolean, desiredRotation);
	}

	public TeleopSwerveCommand getTeleopAimToPositionCommand(final Joystick Controller,
			final double DesiredRotation)
	{
		final JoystickButton BumperRight = new JoystickButton(Controller,
				LogitechControllerButtons.bumperRight);

		return new TeleopAimSwerveToPositionCommand(this, () -> -Controller.getY(),
				() -> -Controller.getX(), BumperRight::getAsBoolean, DesiredRotation);
	}

	/**
	 * @param Controller that controls the swerve drive
	 * @param Rotation   supplier for the rotation of the swerve drive
	 * @return the default command for the swerve drive that allows driver control except for
	 *         rotation
	 */
	public TeleopSwerveCommand getTeleopControlledRotationCommand(final Joystick Controller,
			final DoubleSupplier Rotation)
	{
		final JoystickButton BumperRight = new JoystickButton(Controller,
				LogitechControllerButtons.bumperRight);

		return new TeleopSwerveCommand(this, () -> -Controller.getY(), () -> -Controller.getX(),
				Rotation, BumperRight::getAsBoolean);
	}


	

	public double getRotationalVelocityToAngle(double angle)
	{
		double referenceAngle = optimizeAngle(getYaw().getRadians(), angle);

		double desiredRotationalVelocity = -autoAimPidController.calculate(referenceAngle, angle);

		SmartDashboard.putNumber("Yaw", referenceAngle);
		SmartDashboard.putNumber("Target Rotation", angle);

		return desiredRotationalVelocity;
	}

	public double optimizeAngle(double referenceAngle, double newAngle)
	{
		while (referenceAngle > newAngle + Math.PI)
		{
			referenceAngle -= 2 * Math.PI;
		}
		while (referenceAngle < newAngle - Math.PI)
		{
			referenceAngle += 2 * Math.PI;
		}

		return referenceAngle;
	}

	/**
	 * @return the velocity of the robot in meters per second
	 */
	public Pose2d getVelocity()
	{
		ChassisSpeeds chassisSpeed = SwerveConstants.SwerveKinematics
				.toChassisSpeeds(getModuleStates());

		return new Pose2d(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond,
				new Rotation2d());
	}

	/** @param estimatedRobotPose estimated robot pose from vision */
	public void updatePoseWithVision(Optional<EstimatedRobotPose> estimatedRobotPose)
	{
		if (estimatedRobotPose.isPresent())
		{
			swervePoseEstimator.addVisionMeasurement(
					estimatedRobotPose.get().estimatedPose.toPose2d(),
					estimatedRobotPose.get().timestampSeconds);
		}
	}

}