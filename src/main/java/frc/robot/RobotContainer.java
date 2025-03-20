package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LedConstants;
import frc.robot.constants.PathSetpoints;
import frc.robot.constants.SuperstructureConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.WristConstants;
import frc.robot.core.Autonomous;
import frc.robot.core.LogitechControllerButtons;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain.PathLocation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private boolean autoRan = false;

    private static RobotContainer instance;

    private final ClimberSubsystem climber;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    private final IntakeSubsystem intake;
    private final LedSubsystem led;
    private final SuperstructureSubsystem superstructure;
    private final CommandSwerveDrivetrain swerve;
    private final VisionSubsystem vision;

    private final ShuffleboardTab shuffleboardTab;

    private final Telemetry logger;

    private Autonomous auto;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        instance = this;

        new PowerDistribution(1, ModuleType.kRev).setSwitchableChannel(true);

        shuffleboardTab = Shuffleboard.getTab(Constants.SHUFFLEBOARD_SUPERSTRUCTURE_TAB);

        logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

        // Instantiate Subsystems
        climber = new ClimberSubsystem();
        elevator = new ElevatorSubsystem();
        arm = new ArmSubsystem();
        wrist = new WristSubsystem();
        intake = new IntakeSubsystem();
        led = new LedSubsystem();
        superstructure = new SuperstructureSubsystem(elevator, arm, wrist, intake, led);
        swerve = TunerConstants.createDrivetrain();
        vision = new VisionSubsystem(swerve);

        // Configure button bindings
        configurePrimaryBindings();
        configureSecondaryBindings();

        // Configure Auto
        auto = new Autonomous(instance);
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
        JoystickButton b = new JoystickButton(joystick, LogitechControllerButtons.b); // scoring at pose override
        JoystickButton x = new JoystickButton(joystick, LogitechControllerButtons.x);
        JoystickButton y = new JoystickButton(joystick, LogitechControllerButtons.y);
        JoystickButton triggerLeft = new JoystickButton(joystick, LogitechControllerButtons.triggerLeft);
        JoystickButton triggerRight = new JoystickButton(joystick, LogitechControllerButtons.triggerRight);
        JoystickButton bumperLeft = new JoystickButton(joystick, LogitechControllerButtons.bumperLeft);
        JoystickButton bumperRight = new JoystickButton(joystick, LogitechControllerButtons.bumperRight);

        Supplier<ChassisSpeeds> desiredChassisSpeeds = () -> { 
            double velocityX = -joystick.getY() * SwerveConstants.MAX_TRANSLATIONAL_VELOCITY;
            double velocityY = -joystick.getX() * SwerveConstants.MAX_TRANSLATIONAL_VELOCITY;
            double velocityAngular = -joystick.getTwist() * SwerveConstants.MAX_ROTATIONAL_VELOCITY;

            if (bumperLeft.getAsBoolean()) {
                velocityX *= -SwerveConstants.PRECISION_MODE_SCALAR;
                velocityY *= -SwerveConstants.PRECISION_MODE_SCALAR;
                velocityAngular *= SwerveConstants.PRECISION_MODE_SCALAR;
            }

            if (Math.abs(velocityX) < SwerveConstants.TRANSLATIONAL_DRIVER_DEADBAND) velocityX = 0;
            if (Math.abs(velocityY) < SwerveConstants.TRANSLATIONAL_DRIVER_DEADBAND) velocityY = 0;
            if (Math.abs(velocityAngular) < SwerveConstants.ROTATIONAL_DRIVER_DEADBAND) velocityAngular = 0;

            if (bumperLeft.getAsBoolean()) {
                return new ChassisSpeeds(velocityX, velocityY, velocityAngular);
            }
            else {
                return ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(velocityX, velocityY, velocityAngular), 
                    swerve.getState().Pose.getRotation().plus(swerve.getOperatorForwardDirection()));
            }
        };

        // Default field relative drive command
        swerve.setDefaultCommand(swerve.driveCommand(desiredChassisSpeeds));

        // Drive to pose commands
        // triggerLeft.whileTrue(swerve.driveToNet(desiredChassisSpeeds));
        triggerLeft.whileTrue(swerve.driveToClosestHumanPlayer(desiredChassisSpeeds));
        triggerRight.whileTrue(swerve.driveToClosestBranch(desiredChassisSpeeds));
        // bumperLeft.whileTrue(swerve.driveToProcessor(desiredChassisSpeeds));
        bumperRight.whileTrue(swerve.driveToClosestReefAlgae(desiredChassisSpeeds));

        // Reset heading
        y.onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        swerve.configureShuffleboard(desiredChassisSpeeds);

    }

    private void configureSecondaryBindings() {
        Joystick joystick = new Joystick(1);

        JoystickButton start = new JoystickButton(joystick, LogitechControllerButtons.start);
        JoystickButton back = new JoystickButton(joystick, LogitechControllerButtons.back);

        POVButton down = new POVButton(joystick, LogitechControllerButtons.down);
        POVButton up = new POVButton(joystick, LogitechControllerButtons.up);
        POVButton left = new POVButton(joystick, LogitechControllerButtons.left);
        POVButton right = new POVButton(joystick, LogitechControllerButtons.right);
        JoystickButton a = new JoystickButton(joystick, LogitechControllerButtons.a);
        JoystickButton b = new JoystickButton(joystick, LogitechControllerButtons.b);
        JoystickButton x = new JoystickButton(joystick, LogitechControllerButtons.x);
        JoystickButton y = new JoystickButton(joystick, LogitechControllerButtons.y);
        JoystickButton bumperLeft = new JoystickButton(joystick, LogitechControllerButtons.bumperLeft);
        JoystickButton bumperRight = new JoystickButton(joystick, LogitechControllerButtons.bumperRight);
        JoystickButton triggerLeft = new JoystickButton(joystick, LogitechControllerButtons.triggerLeft);
        JoystickButton triggerRight = new JoystickButton(joystick, LogitechControllerButtons.triggerRight);

        Supplier<Boolean> overrideAtPose = () -> new JoystickButton(new Joystick(0), LogitechControllerButtons.b).getAsBoolean();
        Supplier<Boolean> overrideNearPose = () -> start.getAsBoolean();
        Supplier<Boolean> isNearAligned = () -> (swerve.isNearAligned() || Robot.isTestMode());
        Supplier<Boolean> isAligned = () -> (swerve.isAligned() || Robot.isTestMode());
        Supplier<Pose2d> getTargetPose = () -> swerve.getTargetPose();

        down.whileTrue(superstructure.scoreCoralL1Command(isNearAligned, isAligned, overrideNearPose, overrideAtPose));
        right.whileTrue(superstructure.scoreCoralL2Command(isNearAligned, isAligned, overrideNearPose, overrideAtPose));
        left.whileTrue(superstructure.scoreCoralL3Command(isNearAligned, isAligned, overrideNearPose, overrideAtPose));
        up.whileTrue(superstructure.scoreCoralL4Command(isNearAligned, isAligned, overrideNearPose, overrideAtPose));
        triggerRight.whileTrue(superstructure.scoreAlgaeProcessorCommand(isNearAligned, isAligned, overrideNearPose, overrideAtPose));
        triggerLeft.whileTrue(superstructure.scoreAlgaeNetCommand(isNearAligned, isAligned, overrideNearPose, overrideAtPose));

        b.whileTrue(superstructure.intakeCoralGroundCommand());
        a.whileTrue(superstructure.intakeCoralHumanPlayerCommand());
        x.whileTrue(superstructure.intakeAlgaeReefCommand(getTargetPose));
        y.whileTrue(superstructure.intakeAlgaeGroundCommand());

        // bumperRight.whileTrue(climber.climbCommand());
        bumperLeft.whileTrue(climber.setVoltageCommand(-8));
        bumperRight.whileTrue(climber.setVoltageCommand(8));

        back.onTrue(superstructure.zeroSuperstructureCommand());

        /*
         * Homing buttons
         */

        // triggerLeft.whileTrue(elevator.setVoltageCommand(1.5));
        // triggerRight.whileTrue(elevator.setVoltageCommand(-1));

        // home.onTrue(arm.setZeroPositionCommand());
        // home.onTrue(elevator.setZeroPositionCommand());

        // bumperLeft.whileTrue(arm.setVoltageCommand(1.5));
        // bumperRight.whileTrue(arm.setVoltageCommand(-1));

        /*
         * Testing buttons
         */

        GenericEntry testingVoltage = shuffleboardTab.add("Testing Voltage", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 2))
            .getEntry();
            
        // triggerLeft.whileTrue(elevator.tuneVoltageCommand(() -> testingVoltage.getDouble(0)));

        // triggerLeft.whileTrue(arm.tuneVoltageCommand(() -> testingVoltage.getDouble(0)));
        // triggerRight.whileTrue(arm.tuneVoltageCommand(() -> -testingVoltage.getDouble(0)));

        // triggerLeft.whileTrue(elevator.setVoltageCommand(1.5));
        // triggerRight.whileTrue(elevator.setVoltageCommand(-1));
        // bumperLeft.onTrue(elevator.setPositionCommand(10));
        // bumperRight.onTrue(elevator.setPositionCommand(40));

        // home.onTrue(arm.setZeroPositionCommand());
        // home.onTrue(elevator.setZeroPositionCommand());

        // bumperLeft.whileTrue(arm.setVoltageCommand(1.5));
        // bumperRight.whileTrue(arm.setVoltageCommand(-1));
        // triggerLeft.onTrue(arm.setPositionCommand(4.5));
        // triggerRight.onTrue(arm.setPositionCommand(20));

        // triggerLeft.whileTrue(wrist.setCurrentCommand(WristConstants.PARALLEL_CURRENT));
        // triggerRight.whileTrue(wrist.setCurrentCommand(WristConstants.PERPENDICULAR_CURRENT));

        // triggerLeft.whileTrue(intake.setVoltageCommand(0.3));
        // triggerRight.whileTrue(intake.setVoltageCommand(0.25));
        // bumperLeft.whileTrue(intake.setVelocityCommand(IntakeConstants.NET_EJECT_VELOCITY));
        // bumperRight.whileTrue(intake.setVelocityCommand(IntakeConstants.PROCESSOR_EJECT_VELOCITY));
        // bumperLeft.whileTrue(intake.setVoltageCommand(testingVoltage.getDouble(0)));

        // triggerLeft.whileTrue(climber.setVoltageCommand(6));
        // triggerRight.whileTrue(climber.setVoltageCommand(-6));

        /*
         * SysId
         */

        // triggerLeft.whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // triggerRight.whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // bumperLeft.whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // bumperRight.whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));

        // triggerLeft.whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // triggerRight.whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // bumperLeft.whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // bumperRight.whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // a.onTrue(Commands.runOnce(() -> SignalLogger.start()));
        // b.onTrue(Commands.runOnce(() -> SignalLogger.stop()));
    }


    public static ShuffleboardTab getShuffleboardTab() {
		return instance.shuffleboardTab;
	}

    public SuperstructureSubsystem getSuperstructure() {
        return superstructure;
    }

    public CommandSwerveDrivetrain getSwerve() {
        return swerve;
    }

    public VisionSubsystem GetVision() {
        return vision;
    }

    // TODO: make autos
    public Command getAutoCommand() {
        return auto.getAutoCommand();
    }

}
