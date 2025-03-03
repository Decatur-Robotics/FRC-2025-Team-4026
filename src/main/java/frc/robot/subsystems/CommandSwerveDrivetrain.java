package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.PathSetpoints;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private Double coralPose;
    private Double coralShift;
    private Double coralRotation;
    private Double algaePose;
    private Double coralDistance;
    private Double algaeDistance;
    private Double robotRotationCoral;
    private Double activeShift;

    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    private Pose2d targetPose = new Pose2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final ModuleRequest moduleRequest = new ModuleRequest()
        .withDriveRequest(DriveRequestType.Velocity)
        .withSteerRequest(SteerRequestType.MotionMagicExpo);

    private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();

    /** Swerve request to apply during robot-centric path following */
    // private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> driveAuto(() -> speeds, () -> feedforwards),
                // setControl(
                //     driveRequest.withSpeeds(speeds)
                //         .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                //         .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                // ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    SwerveConstants.AUTO_TRANSLATIONAL_CONSTANTS,
                    // PID constants for rotation
                    SwerveConstants.AUTO_ROTATIONAL_CONSTANTS
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );

            setpointGenerator = new SwerveSetpointGenerator(
            config, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
            Units.rotationsToRadians(SwerveConstants.MAX_ROTATIONAL_VELOCITY) 
        );
        ChassisSpeeds currentSpeeds = getState().Speeds;
        SwerveModuleState[] currentStates = getState().ModuleStates; // Method to get the current swerve module states
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));

        } 
        catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public void configureShuffleboard(Supplier<ChassisSpeeds> speeds){
        ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

        tab.addDouble("Target Swerve Speed X", () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), this.getState().Pose.getRotation().plus(getOperatorForwardDirection())).vxMetersPerSecond);
        tab.addDouble("Target Swerve Speed Y", () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), this.getState().Pose.getRotation().plus(getOperatorForwardDirection())).vyMetersPerSecond);

        tab.addDouble("Actual Swerve Speed X", () -> getState().Speeds.vxMetersPerSecond);
        tab.addDouble("Actual Swerve Speed Y", () -> getState().Speeds.vyMetersPerSecond);

        tab.addDouble("Actual Swerve Pose X", () -> getState().Pose.getX());
        tab.addDouble("Actual Swerve Pose Y", () -> getState().Pose.getY());
        tab.addDouble("Actual Swerve Rotation", () -> getState().Pose.getRotation().getDegrees());

        tab.addDouble("Target Swerve Pose X", () -> targetPose.getX());
        tab.addDouble("Target Swerve Pose Y", () -> targetPose.getY());
        tab.addDouble("Target Swerve Rotation", () -> targetPose.getRotation().getDegrees());

        tab.addDouble("Target Module 0 Velocity", () -> getState().ModuleTargets[0].speedMetersPerSecond);
        tab.addDouble("Actual Module 0 Velocity", () -> getState().ModuleStates[0].speedMetersPerSecond);
        tab.addDouble("Target Module 1 Velocity", () -> getState().ModuleTargets[1].speedMetersPerSecond);
        tab.addDouble("Actual Module 1 Velocity", () -> getState().ModuleStates[1].speedMetersPerSecond);
        tab.addDouble("Target Module 2 Velocity", () -> getState().ModuleTargets[2].speedMetersPerSecond);
        tab.addDouble("Actual Module 2 Velocity", () -> getState().ModuleStates[2].speedMetersPerSecond);
        tab.addDouble("Target Module 3 Velocity", () -> getState().ModuleTargets[3].speedMetersPerSecond);
        tab.addDouble("Actual Module 3 Velocity", () -> getState().ModuleStates[3].speedMetersPerSecond);
    }
    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Returns a command that applies specified a swerve setpoint from specified field relative chassis speeds to this swerve drivetrain.
     * 
     * @param speeds Function returning the field relative chassis speeds to apply
     * @return Command to run
     */
    public Command driveFieldRelative(Supplier<ChassisSpeeds> speeds) {
        return driveRobotRelative(() -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), this.getState().Pose.getRotation().plus(getOperatorForwardDirection())));
    }

    /**
     * Returns a command that applies specified a swerve setpoint from specified robot relative chassis speeds to this swerve drivetrain.
     * 
     * @param speeds Function returning the robot relative chassis speeds to apply
     * @return Command to run
     */
    public Command driveRobotRelative(Supplier<ChassisSpeeds> speeds) {
        return run(() -> {
            // Note: it is important to not discretize speeds before or after
            // using the setpoint generator, as it will discretize them for you
            previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                speeds.get(), // The desired target speeds
                0.02 // The loop time of the robot code, in seconds
            );

            setControl(driveRequest.withSpeeds(previousSetpoint.robotRelativeSpeeds()));
        });
    }

    /**
     * Returns a command that applies specified a swerve setpoint from specified robot relative chassis speeds to this swerve drivetrain.
     * For PathPlanner.
     * 
     * @param speeds Function returning the robot relative chassis speeds to apply
     * @return Command to run
     */
    public Command driveAuto(Supplier<ChassisSpeeds> speeds, Supplier<DriveFeedforwards> feedforwards) {
        return run(() -> {
            // Note: it is important to not discretize speeds before or after
            // using the setpoint generator, as it will discretize them for you
            previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                speeds.get(), // The desired target speeds
                0.02 // The loop time of the robot code, in seconds
            );

            setControl(driveRequest.withSpeeds(previousSetpoint.robotRelativeSpeeds())
                .withWheelForceFeedforwardsX(feedforwards.get().robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.get().robotRelativeForcesYNewtons()));
        });
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public Pose2d getClosestPoseFromArrayAllianceRelative(Pose2d[] poses) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) {
            for (Pose2d pose : poses) {
                pose = pose.rotateAround(PathSetpoints.FIELD_CENTER, PathSetpoints.FLIP_ROTATION);
            }
        }

        return getClosestPoseFromArray(poses);
    }

    public Pose2d getClosestPoseFromArray(Pose2d[] poses) {
        double distanceToClosestPose;
        Pose2d closestPose;

        distanceToClosestPose = poses[0].getTranslation().getDistance(getState().Pose.getTranslation());
        closestPose = poses[0];

        for (Pose2d pose : poses) {
            double distanceToPose = pose.getTranslation().getDistance(getState().Pose.getTranslation());
            if (distanceToPose < distanceToClosestPose) {
                distanceToClosestPose = distanceToPose;
                closestPose = pose;
            }
        }

        return closestPose;
    }

    public Command driveToPose(Supplier<ChassisSpeeds> speeds, Pose2d targetPose) {
        this.targetPose = targetPose;
        
        return run(() -> {
            if (speeds.get().vxMetersPerSecond == 0 && speeds.get().vyMetersPerSecond == 0) {
                speeds.get().vxMetersPerSecond = SwerveConstants.TRANSLATIONAL_CONTROLLER.calculate(
                    getState().Pose.getX(), targetPose.getX());
                speeds.get().vyMetersPerSecond = SwerveConstants.TRANSLATIONAL_CONTROLLER.calculate(
                    getState().Pose.getY(), targetPose.getY());
            }

            if (speeds.get().omegaRadiansPerSecond == 0) {
                speeds.get().omegaRadiansPerSecond = SwerveConstants.ROTATIONAL_CONTROLLER.calculate(
                    getState().Pose.getRotation().getRadians(), targetPose.getRotation().getRadians());
            }

            driveFieldRelative(speeds);
        })
        .finallyDo(() -> this.targetPose = null);
    }

    public Command driveToClosestBranch(Supplier<ChassisSpeeds> speeds) {
        Pose2d pose = getClosestPoseFromArrayAllianceRelative(PathSetpoints.CORAL_SCORING_POSES);

        return driveToPose(speeds, pose);
    }

    public Command driveToClosestReefAlgae(Supplier<ChassisSpeeds> speeds) {
        Pose2d pose = getClosestPoseFromArrayAllianceRelative(PathSetpoints.REEF_ALGAE_POSES);
        
        return driveToPose(speeds, pose);
    }

    public Command driveToProcessor(Supplier<ChassisSpeeds> speeds) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        Pose2d pose = PathSetpoints.PROCESSOR;

        if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) 
            pose = pose.rotateAround(PathSetpoints.FIELD_CENTER, PathSetpoints.FLIP_ROTATION);

        return driveToPose(speeds, pose);
    }

    public Command driveToNet(Supplier<ChassisSpeeds> speeds) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        Pose2d pose = PathSetpoints.NET;

        if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) 
            pose = pose.rotateAround(PathSetpoints.FIELD_CENTER, PathSetpoints.FLIP_ROTATION);
        
        return driveToPose(speeds, pose);
    }

    public boolean isAtTargetPose() {
        if (targetPose.equals(null)) return false;
        
        boolean isAtTargetX = getState().Pose.getX() - targetPose.getX() < SwerveConstants.TRANSLATIONAL_ERROR_MARGIN;
        boolean isAtTargetY = getState().Pose.getY() - targetPose.getY() < SwerveConstants.TRANSLATIONAL_ERROR_MARGIN;
        boolean isAtTargetRotation = getState().Pose.getRotation().getRadians() - targetPose.getRotation().getRadians() < SwerveConstants.ROTATIONAL_ERROR_MARGIN;

        return isAtTargetX && isAtTargetY && isAtTargetRotation;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    // TODO: Some edits will need to be made to these methods in the future
    // I can explain these at some point, cant think of a good name
    // public void orbitWowCoolThing() {
    //     robotRotationCoral = Math.asin(coralPose/coralDistance);
    //     activeShift = coralDistance;
    //     //will be added with button bindings, theres probably a better way to do this
    //    // while (buttonPressed) {
    //      activeShift = activeShift - 0.01;
    //     coralShift = robotRotationCoral*Math.sin(coralDistance/(2*Math.PI)*activeShift) + robotRotationCoral;
    //     m_rotationCharacterization.withRotationalRate(coralShift);
        
    //   //  }
    // }

    
    // public Command coralMeta(){
    //   return AutoBuilder.pathfindToPose(new Pose2d(coralPose + 0.5, Math.pow(coralDistance, 2) - Math.pow(coralPose, 2), new Rotation2d(coralRotation)), SwerveConstants.CONSTRAINTS, 0);  
    // }

}
