package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.PathSetpoints;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    
    private Pose2d initialPose;
    private boolean hasAppliedInitialPose;

    private DoubleSubscriber coralRotationSubscriber;
    private double coralRotation;

    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    private Pose2d targetPose = null;

    private StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Target Pose", Pose2d.struct).publish();

    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Robot Pose", Pose2d.struct).publish();

    private PIDController translationalController = new PIDController(
        5.75, 0, 0.2);
        // 5.25, 0, 0.3); 
    private PIDController rotationalController = new PIDController(
        6.25, 0, 0.3);

    public enum PathLocation {
        None(List.of()),
        Reef(List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)),
        Processor(List.of(3, 16)),
        Net(List.of(4, 5, 14, 15)),
        HumanPlayer(List.of(1, 2, 12, 13));

        private final List<Integer> apriltagIds;

        private PathLocation(List<Integer> apriltagIds) {
            this.apriltagIds = apriltagIds;
        }

        public List<Integer> getApriltagIds() {
            return this.apriltagIds;
        }
    }

    private PathLocation targetPoseLocation = PathLocation.None;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

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

        DoubleTopic coralTopic = NetworkTableInstance.getDefault().getDoubleTopic("CoralRotation");

        coralRotationSubscriber = coralTopic.subscribe(0.0);


        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();

        rotationalController.enableContinuousInput(0, 2*Math.PI);

        hasAppliedInitialPose = false;
        initialPose = Pose2d.kZero;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings(); // SwerveConstants.CONFIG;
            AutoBuilder.configure(
                () -> this.getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> this.getState().Speeds, // Supplier of current robot speeds
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

        tab.addDouble("Target Swerve Speed X", () -> speeds.get().vxMetersPerSecond);
        tab.addDouble("Target Swerve Speed Y", () -> speeds.get().vyMetersPerSecond);
        tab.addDouble("Target Swerve Speed Rotation", () -> speeds.get().omegaRadiansPerSecond);

        tab.addDouble("Actual Swerve Speed X", () -> getState().Speeds.vxMetersPerSecond);
        tab.addDouble("Actual Swerve Speed Y", () -> getState().Speeds.vyMetersPerSecond);

        tab.addDouble("Actual Swerve Pose X", () -> getState().Pose.getX());
        tab.addDouble("Actual Swerve Pose Y", () -> getState().Pose.getY());
        tab.addDouble("Actual Swerve Rotation", () -> getState().Pose.getRotation().getDegrees());

        tab.addDouble("Target Module 0 Velocity", () -> getState().ModuleTargets[0].speedMetersPerSecond);
        tab.addDouble("Actual Module 0 Velocity", () -> getState().ModuleStates[0].speedMetersPerSecond);
        tab.addDouble("Target Module 1 Velocity", () -> getState().ModuleTargets[1].speedMetersPerSecond);
        tab.addDouble("Actual Module 1 Velocity", () -> getState().ModuleStates[1].speedMetersPerSecond);
        tab.addDouble("Target Module 2 Velocity", () -> getState().ModuleTargets[2].speedMetersPerSecond);
        tab.addDouble("Actual Module 2 Velocity", () -> getState().ModuleStates[2].speedMetersPerSecond);
        tab.addDouble("Target Module 3 Velocity", () -> getState().ModuleTargets[3].speedMetersPerSecond);
        tab.addDouble("Actual Module 3 Velocity", () -> getState().ModuleStates[3].speedMetersPerSecond);

        tab.addDouble("Module 0 Current", () -> getModule(0).getDriveMotor().getStatorCurrent().getValueAsDouble());
    
        tab.addBoolean("At Target Pose", () -> isAtTargetPose());
        tab.addBoolean("Aligned", () -> isAligned());
        
        tab.addBoolean("Near Target Pose", () -> isNearTargetPose());
        tab.addBoolean("Near Aligned", () -> isNearAligned());

        tab.addDouble("Operator Rotation", () -> getOperatorForwardDirection().getDegrees());

        tab.addDouble("Target Pose X", () -> {
            if (!(targetPose == null)) return targetPose.getX();
            return -1000;
        });
    }

    public void setInitialPose(Pose2d initialPose) {
        hasAppliedInitialPose = false;
        this.initialPose = initialPose;
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

        if (!hasAppliedInitialPose) {
            resetPose(initialPose);
            hasAppliedInitialPose = true;
        }

        coralRotation = coralRotationSubscriber.get();

        if (!(targetPose == null)) targetPosePublisher.set(targetPose);
        posePublisher.set(getState().Pose);
    }

    /**
     * Returns a command that applies specified a swerve setpoint from specified field relative chassis speeds to this swerve drivetrain.
     * 
     * @param speeds Function returning the robot relative chassis speeds to apply
     * @return Command to run
     */
    public Command driveCommand(Supplier<ChassisSpeeds> speeds) {
        return Commands.run(() -> driveRobotRelative(speeds.get()), this);
    }

    /**
     * Returns a command that applies specified a swerve setpoint from specified robot relative chassis speeds to this swerve drivetrain.
     * 
     * @param speeds Function returning the robot relative chassis speeds to apply
     * @return Command to run
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, // The previous setpoint
            speeds, // The desired target speeds
            0.02 // The loop time of the robot code, in seconds
        );

        setControl(driveRequest.withSpeeds(previousSetpoint.robotRelativeSpeeds()));
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getState().Pose.getRotation().plus(getOperatorForwardDirection())));
    }

    public void resetPreviousSetpoint() {
        ChassisSpeeds currentSpeeds = getState().Speeds;
        SwerveModuleState[] currentStates = getState().ModuleStates; // Method to get the current swerve module states
        previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(4));
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

    /**
     * Returns a command that applies specified a swerve setpoint from specified robot relative chassis speeds to this swerve drivetrain.
     * For PathPlanner.
     * 
     * @param speeds Function returning the robot relative chassis speeds to apply
     * @return Command to run
     */
    public Command driveAuto(Supplier<ChassisSpeeds> speeds, Supplier<DriveFeedforwards> feedforwards) {
        return run(() -> {
            // // Note: it is important to not discretize speeds before or after
            // // using the setpoint generator, as it will discretize them for you
            // previousSetpoint = setpointGenerator.generateSetpoint(
            //     previousSetpoint, // The previous setpoint
            //     speeds.get(), // The desired target speeds
            //     0.02 // The loop time of the robot code, in seconds
            // );

            // setControl(driveRequest.withSpeeds(previousSetpoint.robotRelativeSpeeds())
            //     .withWheelForceFeedforwardsX(feedforwards.get().robotRelativeForcesXNewtons())
            //     .withWheelForceFeedforwardsY(feedforwards.get().robotRelativeForcesYNewtons()));

            System.out.println("driving auto"); 

            driveRobotRelative(speeds.get());
        });
    }

    public PathLocation getTargetPoseLocation() {
        return targetPoseLocation;
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

    public Command driveToPoseTeleop(Supplier<ChassisSpeeds> speeds, Supplier<Pose2d> targetPose,
            PathLocation targetPoseLocation) {
        return Commands.run(() -> driveToPose(speeds, targetPose, targetPoseLocation, SwerveConstants.MAX_DRIVE_TO_POSE_SPEED), this)
            .finallyDo(() -> {
                this.targetPose = null;
                this.targetPoseLocation = PathLocation.None;
            });
    }

    public Command driveToPoseAuto(Pose2d targetPose,
            PathLocation targetPoseLocation) {
        return Commands.run(() -> driveToPose(() -> new ChassisSpeeds(0, 0, 0), 
                    () -> targetPose, targetPoseLocation, SwerveConstants.MAX_DRIVE_TO_POSE_SPEED), this)
            .finallyDo(() -> {
                this.targetPose = null;
                this.targetPoseLocation = PathLocation.None;
            });
    }

    public Command driveToHumanPlayerFromReefBacksideAuto(Pose2d targetPose, Pose2d startingPose) {
        Supplier<Pose2d> humanPlayerPose = () -> {
            double yError = Math.abs(getState().Pose.getY() - targetPose.getY());

            double offset = 0.5;

            Rotation2d rotation = startingPose.getRotation();

            if (yError < AutoConstants.Y_TOLERANCE) {
                offset = 0;
                rotation = targetPose.getRotation();
            }

            if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
                return new Pose2d(targetPose.getX() + offset, targetPose.getY(), rotation); 
            }
            else {
                return new Pose2d(targetPose.getX() - offset, targetPose.getY(), rotation);
            }
        };

        return Commands.deadline(Commands.waitUntil(() -> isAligned()), 
                Commands.run(() -> driveToPose(() -> new ChassisSpeeds(0, 0, 0), 
                    humanPlayerPose, PathLocation.HumanPlayer, SwerveConstants.MAX_DRIVE_TO_POSE_SPEED), this))
            .finallyDo(() -> {
                this.targetPose = null;
                this.targetPoseLocation = PathLocation.None;
            });
    }

    /**
     * Drive to a target pose with speed overrides
     * 
     * @param speeds Desired robot relative speeds from the driver
     * @param targetPose Function returning pose to drive to
     * @param targetPoseLocation Field location of target pose
     * @return
     */
    public void driveToPose(Supplier<ChassisSpeeds> speeds, Supplier<Pose2d> targetPose,
            PathLocation targetPoseLocation, double maxSpeed) {
        this.targetPose = targetPose.get();
        this.targetPoseLocation = targetPoseLocation;

        double targetRotation = speeds.get().omegaRadiansPerSecond;

        if (speeds.get().omegaRadiansPerSecond == 0) {
            targetRotation = rotationalController.calculate(
                getState().Pose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());
        }

        if (speeds.get().vxMetersPerSecond == 0 && speeds.get().vyMetersPerSecond == 0) {
            double targetTranslation = translationalController.calculate(
                0, getState().Pose.getTranslation().getDistance(this.targetPose.getTranslation()));

            // if (targetTranslation > maxSpeed) targetTranslation = maxSpeed;

            ChassisSpeeds newSpeeds = new ChassisSpeeds(
                isAligned() ? 0 : targetTranslation, 
                0, 
                isAligned() ? 0 : targetRotation);

            Rotation2d travelRotation = this.targetPose.getTranslation().minus(getState().Pose.getTranslation()).getAngle();

            this.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                newSpeeds, getState().Pose.getRotation().minus(travelRotation)));
        }
        else {
            ChassisSpeeds newSpeeds = new ChassisSpeeds(speeds.get().vxMetersPerSecond, speeds.get().vyMetersPerSecond, targetRotation);

            this.driveRobotRelative(newSpeeds);
        }
    }

    public Command driveToClosestBranch(Supplier<ChassisSpeeds> speeds) {
        Supplier<Pose2d> pose = () -> getClosestPoseFromArray(PathSetpoints.CORAL_SCORING_POSES);

        return driveToPoseTeleop(speeds, pose, PathLocation.Reef);
    }

    public Command driveToClosestReefAlgae(Supplier<ChassisSpeeds> speeds) {
        Supplier<Pose2d> pose = () -> getClosestPoseFromArray(PathSetpoints.REEF_ALGAE_POSES);
        
        return driveToPoseTeleop(speeds, pose, PathLocation.Reef);
    }

    public Command driveToProcessor(Supplier<ChassisSpeeds> speeds) {
        Supplier<Pose2d> pose = () -> {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue)) return PathSetpoints.BLUE_PROCESSOR;
            return PathSetpoints.RED_PROCESSOR;
        };

        return driveToPoseTeleop(speeds, pose, PathLocation.Processor);
    }

    public Command driveToNet(Supplier<ChassisSpeeds> speeds) {
        Supplier<Pose2d> pose = () -> {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue)) return PathSetpoints.BLUE_NET;
            return PathSetpoints.RED_NET;
        };
        
        return driveToPoseTeleop(speeds, pose, PathLocation.Net);
    }

    public Command driveToClosestHumanPlayer(Supplier<ChassisSpeeds> speeds) {
        Supplier<Pose2d> pose = () -> getClosestPoseFromArray(PathSetpoints.HUMAN_PLAYER_POSES);
        
        return driveToPoseTeleop(speeds, pose, PathLocation.Reef);
    }

    public boolean isAtTargetPose() {
        if (targetPose == null) return false;
        
        boolean isAtTargetX = Math.abs(translationalController.getError()) < SwerveConstants.TRANSLATIONAL_ALIGNED_ERROR_MARGIN;
        boolean isAtTargetRotation = Math.abs(rotationalController.getError()) < SwerveConstants.ROTATIONAL_ALIGNED_ERROR_MARGIN;

        return isAtTargetX && isAtTargetRotation;
    }

    public boolean isNearTargetPose() {
        if (targetPose == null) return false;
        
        boolean isNearTargetX = Math.abs(translationalController.getError()) < SwerveConstants.TRANSLATIONAL_NEAR_ALIGNED_ERROR_MARGIN;
        boolean isNearTargetRotation = Math.abs(rotationalController.getError()) < SwerveConstants.ROTATIONAL_NEAR_ALIGNED_ERROR_MARGIN;

        return isNearTargetX && isNearTargetRotation;
    }

    public boolean isAligned() {
        boolean velocityAligned = true; 

        for (SwerveModuleState module : getState().ModuleStates) {
            if (module.speedMetersPerSecond > SwerveConstants.VELOCITY_ALIGNED_ERROR_MARGIN) 
                velocityAligned = false;
        }

        return isAtTargetPose() && velocityAligned;
    }

    public boolean isNearAligned() {
        boolean velocityNearAligned = true;

        for (SwerveModuleState module : getState().ModuleStates) {
            if (module.speedMetersPerSecond > SwerveConstants.VELOCITY_NEAR_ALIGNED_ERROR_MARGIN) 
                velocityNearAligned = false;
        }

        return isNearTargetPose() && velocityNearAligned;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public Command nullTargetPose() {
        return Commands.runOnce(() -> targetPose = null, this);
    }

    public Command alignClimb() {
        return Commands.sequence(
            Commands.runOnce(() -> this.targetPose = getClosestPoseFromArray(PathSetpoints.CAGE_STAGE_POSES)),
            Commands.deadline(Commands.waitUntil(() -> isAligned()), 
                driveToPoseTeleop(() -> new ChassisSpeeds(), () -> targetPose, PathLocation.None)),
            Commands.runOnce(() -> this.targetPose = getClosestPoseFromArray(PathSetpoints.CAGE_ALIGN_POSES)),
            Commands.run(() -> driveToPose(() -> new ChassisSpeeds(), () -> targetPose, PathLocation.None, 1))
        )
        .finallyDo(() -> this.targetPose = null);
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

    /* 
     * SysId Commands
     */

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

    public Command driveToCoral(Supplier<ChassisSpeeds> speeds){
        return Commands.run(() -> 
        {
            double targetRotation = speeds.get().omegaRadiansPerSecond;
            double targetX = speeds.get().vxMetersPerSecond;
            double targetY = speeds.get().vyMetersPerSecond;

            if(speeds.get().omegaRadiansPerSecond == 0) {
                targetRotation = rotationalController.calculate(getState().Pose.getRotation().getRadians(), coralRotation);
            }
            if(speeds.get().vxMetersPerSecond == 0 && speeds.get().vyMetersPerSecond == 0) {
                targetX = -1;
            }

            driveRobotRelative(new ChassisSpeeds(targetX, targetY, targetRotation));
        });
    }
}
