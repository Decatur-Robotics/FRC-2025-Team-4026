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

    private SendableChooser<AutoSide> autoSideChooser;
    private SendableChooser<AutoLevel> autoLevelChooser;

    private enum AutoSide {
        Left("Left Side"), Center("Center"), Right("Right Side");

        private final String autoName;

        private AutoSide(String autoName) {
            this.autoName = autoName;
        }
    }

    private enum AutoLevel {
        L1("L1"), L4("L4");

        private final String autoName;

        private AutoLevel(String autoName) {
            this.autoName = autoName;
        }
    }

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

        autoSideChooser = new SendableChooser<>();
        autoSideChooser.setDefaultOption(AutoSide.Center.autoName, AutoSide.Center);
        autoSideChooser.addOption(AutoSide.Left.autoName, AutoSide.Left);
        autoSideChooser.addOption(AutoSide.Center.autoName, AutoSide.Center);
        autoSideChooser.addOption(AutoSide.Right.autoName, AutoSide.Right);

        autoLevelChooser = new SendableChooser<>();
        autoLevelChooser.setDefaultOption(AutoLevel.L4.autoName, AutoLevel.L4);
        autoLevelChooser.addOption(AutoLevel.L1.autoName, AutoLevel.L1);
        autoLevelChooser.addOption(AutoLevel.L4.autoName, AutoLevel.L4);

        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

        autoTab.add(autoSideChooser);
        autoTab.add(autoLevelChooser);

        autoTab.addBoolean("Auto Ran", () -> autoRan);
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
        JoystickButton b = new JoystickButton(joystick, LogitechControllerButtons.b);
        JoystickButton triggerLeft = new JoystickButton(joystick, LogitechControllerButtons.triggerLeft);
        JoystickButton triggerRight = new JoystickButton(joystick, LogitechControllerButtons.triggerRight);
        JoystickButton bumperLeft = new JoystickButton(joystick, LogitechControllerButtons.bumperLeft);
        JoystickButton bumperRight = new JoystickButton(joystick, LogitechControllerButtons.bumperRight);

        Supplier<ChassisSpeeds> desiredChassisSpeeds = () -> { 
            double velocityX = -joystick.getY() * SwerveConstants.MAX_TRANSLATIONAL_VELOCITY;
            double velocityY = -joystick.getX() * SwerveConstants.MAX_TRANSLATIONAL_VELOCITY;
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

        // triggerLeft.onFalse(swerve.nullTargetPose());
        // triggerRight.onFalse(swerve.nullTargetPose());
        // bumperLeft.onFalse(swerve.nullTargetPose());
        // bumperRight.onFalse(swerve.nullTargetPose());

        // Reset heading
        a.onTrue(swerve.runOnce(() -> swerve.resetRotation(swerve.getOperatorForwardDirection())));

        swerve.configureShuffleboard(desiredChassisSpeeds);

    }

    private void configureSecondaryBindings() {
        Joystick joystick = new Joystick(1);

        JoystickButton start = new JoystickButton(joystick, LogitechControllerButtons.start);
        JoystickButton home = new JoystickButton(joystick, LogitechControllerButtons.home);

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

        Supplier<Boolean> overrideAtPose = () -> new JoystickButton(new Joystick(0), LogitechControllerButtons.y).getAsBoolean();
        Supplier<Boolean> overrideNearPose = () -> new JoystickButton(new Joystick(1), LogitechControllerButtons.start).getAsBoolean();
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
        a.whileTrue(superstructure.intakeAlgaeGroundCommand());
        x.whileTrue(superstructure.intakeAlgaeReefLowCommand());
        y.whileTrue(superstructure.intakeAlgaeReefHighCommand());

        // bumperRight.whileTrue(climber.climbCommand());
        // bumperLeft.whileTrue(climber.setVoltageCommand(-8));
        // bumperRight.whileTrue(climber.setVoltageCommand(8));

        // bumperLeft.onTrue(superstructure.zeroSuperstructureCommand());

        /*
         * Testing buttons
         */

        // GenericEntry elevatorVoltage = shuffleboardTab.add("Elevator Voltage", 0)
        //     .withWidget(BuiltInWidgets.kNumberSlider)
        //     .withProperties(Map.of("min", -1, "max", 1))
        //     .getEntry();
            
        // triggerLeft.whileTrue(elevator.tuneVoltageCommand(() -> elevatorVoltage.getDouble(0)));

        // GenericEntry armVoltage = shuffleboardTab.add("Arm Voltage", 0)
        //     .withWidget(BuiltInWidgets.kNumberSlider)
        //     .withProperties(Map.of("min", 0, "max", 1.5))
        //     .getEntry();

        // triggerLeft.whileTrue(arm.tuneVoltageCommand(() -> armVoltage.getDouble(0)));
        // triggerRight.whileTrue(arm.tuneVoltageCommand(() -> -armVoltage.getDouble(0)));

        // triggerLeft.whileTrue(elevator.setVoltageCommand(2));
        // triggerRight.whileTrue(elevator.setVoltageCommand(-1));
        // // bumperLeft.onTrue(elevator.setPositionCommand(10));
        // // bumperRight.onTrue(elevator.setPositionCommand(40));

        // home.onTrue(arm.setZeroPositionCommand());
        // home.onTrue(elevator.setZeroPositionCommand());

        // bumperLeft.whileTrue(arm.setVoltageCommand(1));
        // bumperRight.whileTrue(arm.setVoltageCommand(-1));
        // triggerLeft.onTrue(arm.setPositionCommand(4.5));
        // triggerRight.onTrue(arm.setPositionCommand(20));

        // triggerLeft.whileTrue(wrist.setCurrentCommand(WristConstants.PARALLEL_CURRENT));
        // triggerRight.whileTrue(wrist.setCurrentCommand(WristConstants.PERPENDICULAR_CURRENT));

        // a.whileTrue(claw.setCurrentCommand(ClawConstants.CLOSED_CURRENT));
        // triggerRight.whileTrue(claw.setCurrentCommand(ClawConstants.OPEN_CURRENT));

        // triggerLeft.whileTrue(intake.setVelocityCommand(IntakeConstants.INTAKE_VELOCITY));
        // triggerRight.whileTrue(intake.setVelocityCommand(IntakeConstants.L1_EJECT_VELOCITY));
        // bumperLeft.whileTrue(intake.setVelocityCommand(IntakeConstants.NET_EJECT_VELOCITY));
        // bumperRight.whileTrue(intake.setVelocityCommand(IntakeConstants.PROCESSOR_EJECT_VELOCITY));
        // triggerLeft.whileTrue(intake.setVoltageCommand(0.23));

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
        return Commands.sequence(swerve.driveToPoseAuto(PathSetpoints.BLUE_REEF_A));
    }

}
