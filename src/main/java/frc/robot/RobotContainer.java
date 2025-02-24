package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.WristConstants;
import frc.robot.core.LogitechControllerButtons;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private static RobotContainer instance;

    private final ClimberSubsystem climber;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    private final ClawSubsystem claw;
    private final IntakeSubsystem intake;
    private final SuperstructureSubsystem superstructure;
    private final CommandSwerveDrivetrain swerve;
    private final VisionSubsystem vision;

    private final ShuffleboardTab shuffleboardTab;

    private final Telemetry logger;

    private Field2d field;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        instance = this;

        new PowerDistribution(1, ModuleType.kRev).setSwitchableChannel(true);

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        shuffleboardTab = Shuffleboard.getTab("Tab 1");

        logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

        // Instantiate Subsystems
        climber = new ClimberSubsystem();
        elevator = new ElevatorSubsystem();
        arm = new ArmSubsystem();
        wrist = new WristSubsystem();
        claw = new ClawSubsystem();
        intake = new IntakeSubsystem();
        superstructure = new SuperstructureSubsystem(elevator, arm, wrist, claw, intake);
        swerve = TunerConstants.createDrivetrain();
        vision = new VisionSubsystem(swerve);

        PathPlannerLogging.setLogCurrentPoseCallback((Pose2d pose) -> {
            field.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((Pose2d pose)-> {
            field.getObject("Target Pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((List<Pose2d> poses) -> {
            field.getObject("Path").setPoses(poses);
        });

        // Configure button bindings
        configurePrimaryBindings();
        configureSecondaryBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configurePrimaryBindings() {
        Joystick joystick = new Joystick(0);

        JoystickButton a = new JoystickButton(joystick, LogitechControllerButtons.a);
        JoystickButton triggerLeft = new JoystickButton(joystick, LogitechControllerButtons.triggerLeft);
        JoystickButton triggerRight = new JoystickButton(joystick, LogitechControllerButtons.triggerRight);
        JoystickButton bumperLeft = new JoystickButton(joystick, LogitechControllerButtons.bumperLeft);
        JoystickButton bumperRight = new JoystickButton(joystick, LogitechControllerButtons.bumperRight);

        Supplier<ChassisSpeeds> desiredChassisSpeeds = () -> { 
            double velocityX = joystick.getY() * SwerveConstants.MAX_TRANSLATIONAL_VELOCITY;
            double velocityY = joystick.getX() * SwerveConstants.MAX_TRANSLATIONAL_VELOCITY;
            double velocityAngular = joystick.getTwist() * SwerveConstants.MAX_ROTATIONAL_VELOCITY;

            if (Math.abs(velocityX) < SwerveConstants.TRANSLATIONAL_DEADBAND) velocityX = 0;
            if (Math.abs(velocityY) < SwerveConstants.TRANSLATIONAL_DEADBAND) velocityY = 0;
            if (Math.abs(velocityAngular) < SwerveConstants.ROTATIONAL_DEADBAND) velocityAngular = 0;

            return new ChassisSpeeds(velocityX, velocityY, velocityAngular);
        };

        swerve.setDefaultCommand(swerve.driveFieldRelative(desiredChassisSpeeds));

        triggerLeft.whileTrue(swerve.driveToNet(desiredChassisSpeeds));
        triggerRight.whileTrue(swerve.driveToClosestBranch(desiredChassisSpeeds));
        bumperLeft.whileTrue(swerve.driveToProcessor(desiredChassisSpeeds));
        bumperRight.whileTrue(swerve.driveToClosestReefAlgae(desiredChassisSpeeds));

        // Reset heading
        a.onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));
    }

    private void configureSecondaryBindings() {
        Joystick joystick = new Joystick(1);

        JoystickButton down = new JoystickButton(joystick, LogitechControllerButtons.down);
        JoystickButton up = new JoystickButton(joystick, LogitechControllerButtons.up);
        JoystickButton left = new JoystickButton(joystick, LogitechControllerButtons.left);
        JoystickButton right = new JoystickButton(joystick, LogitechControllerButtons.right);
        JoystickButton a = new JoystickButton(joystick, LogitechControllerButtons.a);
        JoystickButton b = new JoystickButton(joystick, LogitechControllerButtons.b);
        JoystickButton x = new JoystickButton(joystick, LogitechControllerButtons.x);
        JoystickButton y = new JoystickButton(joystick, LogitechControllerButtons.y);
        JoystickButton bumperLeft = new JoystickButton(joystick, LogitechControllerButtons.bumperLeft);
        JoystickButton bumperRight = new JoystickButton(joystick, LogitechControllerButtons.bumperRight);
        JoystickButton triggerLeft = new JoystickButton(joystick, LogitechControllerButtons.triggerLeft);
        JoystickButton triggerRight = new JoystickButton(joystick, LogitechControllerButtons.triggerRight);

        // Supplier<Boolean> overrideLineUp = () -> new JoystickButton(new Joystick(0), LogitechControllerButtons.y).getAsBoolean();
        // Supplier<Boolean> isAtTargetPose = () -> swerve.isAtTargetPose();
        // Supplier<Pose2d> getTargetPose = () -> swerve.getTargetPose();

        // up.whileTrue(superstructure.scoreCoralL1Command(isAtTargetPose, overrideLineUp));
        // left.whileTrue(superstructure.scoreCoralL2Command(isAtTargetPose, overrideLineUp));
        // right.whileTrue(superstructure.scoreCoralL3Command(isAtTargetPose, overrideLineUp));
        // down.whileTrue(superstructure.scoreCoralL4Command(isAtTargetPose, overrideLineUp));
        // triggerRight.whileTrue(superstructure.scoreAlgaeProcessorCommand(isAtTargetPose, overrideLineUp));
        // triggerLeft.whileTrue(superstructure.scoreAlgaeNetCommand(isAtTargetPose, overrideLineUp));

        // b.whileTrue(superstructure.intakeCoralGroundCommand());
        // y.whileTrue(superstructure.intakeCoralHumanPlayerCommand());
        // a.whileTrue(superstructure.intakeAlgaeGroundCommand());
        // x.whileTrue(superstructure.intakeAlgaeReefCommand(getTargetPose));

        // bumperRight.whileTrue(climber.climbCommand());

        // bumperLeft.onTrue(superstructure.zeroSuperstructureCommand());

        /*
         * Testing buttons
         */

        triggerLeft.whileTrue(elevator.setVoltageCommand(1));
        triggerRight.whileTrue(elevator.setVoltageCommand(-1));
        // triggerRight.onTrue(elevator.setPositionCommand(0));

        // triggerLeft.whileTrue(arm.setVoltageCommand(0));
        // triggerRight.onTrue(arm.setPositionCommand(0));

        // triggerLeft.whileTrue(wrist.setCurrentCommand(WristConstants.PARALLEL_CURRENT));
        // triggerRight.whileTrue(wrist.setCurrentCommand(WristConstants.PERPENDICULAR_CURRENT));

        // triggerLeft.whileTrue(claw.setCurrentCommand(ClawConstants.CLOSED_CURRENT));
        // triggerRight.whileTrue(claw.setCurrentCommand(ClawConstants.OPEN_CURRENT));

        // triggerLeft.whileTrue(intake.setVelocityCommand(IntakeConstants.INTAKE_VELOCITY));
        // triggerRight.whileTrue(intake.setVelocityCommand(IntakeConstants.L1_EJECT_VELOCITY));
        // bumperLeft.whileTrue(intake.setVelocityCommand(IntakeConstants.NET_EJECT_VELOCITY));
        // bumperRight.whileTrue(intake.setVelocityCommand(IntakeConstants.PROCESSOR_EJECT_VELOCITY));
        // a.whileTrue(intake.setVoltageCommand(0));
    }

    public static ShuffleboardTab getShuffleboardTab() {
		return instance.shuffleboardTab;
	}
}
