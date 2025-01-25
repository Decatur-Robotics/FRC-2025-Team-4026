package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
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
    private final EndEffectorSubsystem endEffector;

    private final ShuffleboardTab shuffleboardTab;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        instance = this;

        shuffleboardTab = Shuffleboard.getTab("Tab 1");

        // Instantiate Subsystems
        elevator = new ElevatorSubsystem();
        arm = new ArmSubsystem();
        wrist = new WristSubsystem();
        claw = new ClawSubsystem();
        intake = new IntakeSubsystem();
        superstructure = new SuperstructureSubsystem(elevator, arm, wrist);
        endEffector = new EndEffectorSubsystem(claw, intake);

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
        
    }

    private void configureSecondaryBindings() {

    }

    public static ShuffleboardTab getShuffleboardTab() {
		return instance.shuffleboardTab;
	}
}
