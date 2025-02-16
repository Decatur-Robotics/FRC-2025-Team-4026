package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.SwerveConstants;
import frc.robot.core.LogitechControllerButtons;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private static RobotContainer instance;

    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;
    private final ClawSubsystem claw;
    private final IntakeSubsystem intake;
    private final SuperstructureSubsystem superstructure;
    private final CommandSwerveDrivetrain swerve;

    private final ShuffleboardTab shuffleboardTab;

    private double MaxSpeed;
    private double MaxAngularRate;

    private final Telemetry logger;

    private Field2d field;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        instance = this;

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        shuffleboardTab = Shuffleboard.getTab("Tab 1");

        MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        logger = new Telemetry(MaxSpeed);

        // Instantiate Subsystems
        elevator = new ElevatorSubsystem();
        arm = new ArmSubsystem();
        wrist = new WristSubsystem();
        claw = new ClawSubsystem();
        intake = new IntakeSubsystem();
        superstructure = new SuperstructureSubsystem(elevator, arm, wrist, claw, intake);
        swerve = TunerConstants.createDrivetrain();

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

        Supplier<ChassisSpeeds> desiredChassisSpeeds = () -> { 
            double velocityX = joystick.getY() * SwerveConstants.MAX_TRANSLATION_VELOCITY;
            double velocityY = joystick.getX() * SwerveConstants.MAX_TRANSLATION_VELOCITY;
            double velocityAngular = joystick.getTwist() * SwerveConstants.MAX_ANGULAR_VELOCITY;

            if (Math.abs(velocityX) < SwerveConstants.TRANSLATIONAL_DEADBAND) velocityX = 0;
            if (Math.abs(velocityY) < SwerveConstants.TRANSLATIONAL_DEADBAND) velocityY = 0;
            if (Math.abs(velocityAngular) < SwerveConstants.ANGULAR_DEADBAND) velocityAngular = 0;

            return new ChassisSpeeds(velocityX, velocityY, velocityAngular);
        };

        JoystickButton a = new JoystickButton(joystick, LogitechControllerButtons.a);
        JoystickButton b = new JoystickButton(joystick, LogitechControllerButtons.b);
        JoystickButton x = new JoystickButton(joystick, LogitechControllerButtons.x);
        JoystickButton y = new JoystickButton(joystick, LogitechControllerButtons.y);


        swerve.setDefaultCommand(swerve.driveFieldRelative(desiredChassisSpeeds));

        b.whileTrue(swerve.driveToClosestBranch(desiredChassisSpeeds));

        // Reset heading
        a.onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));
        
    }

    private void configureSecondaryBindings() {

    }

    public static ShuffleboardTab getShuffleboardTab() {
		return instance.shuffleboardTab;
	}
}
