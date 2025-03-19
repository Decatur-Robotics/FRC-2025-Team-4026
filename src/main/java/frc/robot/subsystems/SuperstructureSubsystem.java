package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LedConstants;
import frc.robot.constants.PathSetpoints;
import frc.robot.constants.SuperstructureConstants;
import frc.robot.util.SuperstructureState;

public class SuperstructureSubsystem extends SubsystemBase {
    
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private WristSubsystem wrist;
    private IntakeSubsystem intake;
    private LedSubsystem led;

    private SuperstructureState targetState;

    public SuperstructureSubsystem(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist,
            IntakeSubsystem intake, LedSubsystem led) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.intake = intake;
        this.led = led;

        targetState = SuperstructureConstants.CORAL_STOWED_STATE.copyInstance();

        configureShuffleboard();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.SHUFFLEBOARD_SUPERSTRUCTURE_TAB);

        tab.addBoolean("Is Superstructure At Target State", () -> isAtTargetState());
    }

    @Override
    public void periodic() {
        
    }

    public void setState(SuperstructureState targetState) {
        this.targetState = targetState.copyInstance();

        elevator.setPosition(targetState.elevatorPosition);
        arm.setPosition(targetState.armPosition);
        wrist.setCurrent(targetState.wristCurrent);
        intake.setVelocity(targetState.intakeVelocity);
    }

    // Is at targets

    public boolean isAtTargetState() {
        return (isElevatorAtTargetPosition() &&
                isArmAtTargetPosition() &&
                isWristAtTargetPosition()) 
                || Robot.isSimulation();
    }

    public boolean isElevatorAtTargetPosition() {
        if (Math.abs(getActualElevatorPosition() - targetState.elevatorPosition) > SuperstructureConstants.ELEVATOR_ERROR_MARGIN) {
            return false;
        }

        return true;
    }

    public boolean isArmAtTargetPosition() {
        if (Math.abs(getActualArmPosition() - targetState.armPosition) > SuperstructureConstants.ARM_ERROR_MARGIN) {
            return false;
        }

        return true;
    }

    public boolean isWristAtTargetPosition() {
        return wrist.isSlammed();
    }

    // Get goal states

    public SuperstructureState getGoalState() {
        return targetState;
    }

    // Get actual states

    public SuperstructureState getActualState() {
        return new SuperstructureState(getActualElevatorPosition(), getActualArmPosition(), 
            getActualWristCurrent(), getActualIntakeVelocity());
    }

    public double getActualElevatorPosition() {
        return elevator.getPosition();
    }

    public double getActualArmPosition() {
        return arm.getTalonPosition();
    }

    public double getActualWristCurrent() {
        return wrist.getCurrent();
    }

    public double getActualIntakeVelocity() {
        return intake.getVelocity();
    }

    // Directly set subsystem targets

    public void setElevatorPosition(double position) {
        targetState.elevatorPosition = position;
        targetState.elevatorPosition = position;
        
        elevator.setPosition(position);
    }

    public void setArmPosition(double position) {
        targetState.armPosition = position;
        targetState.armPosition = position;
        
        arm.setPosition(position);
    }

    public void setWristCurrent(double current) {
        targetState.wristCurrent = current;
        targetState.wristCurrent = current;
        
        wrist.setCurrent(current);
    }

    public void setIntakeVelocity(double velocity) {
        targetState.intakeVelocity = velocity;
        targetState.intakeVelocity = velocity;
        
        intake.setVelocity(velocity);
    }

    // Reset offsets commands

    public Command zeroSuperstructureCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureConstants.CORAL_STOWED_STATE), 
                elevator, arm, wrist, intake),
            Commands.parallel(
                elevator.zeroCommand(),
                arm.zeroCommand()))
            .finallyDo(() -> setState(SuperstructureConstants.CORAL_STOWED_STATE));
    }

    // Intaking commands

    public Command intakeCoralGroundCommand() {
        return intakeCommand(() -> SuperstructureConstants.CORAL_GROUND_INTAKING_STATE, 
            SuperstructureConstants.CORAL_STOWED_STATE);
    }

    public Command intakeCoralHumanPlayerCommand() {
        return intakeCommand(() -> SuperstructureConstants.CORAL_HUMAN_PLAYER_INTAKING_STATE, 
            SuperstructureConstants.CORAL_STOWED_STATE);
    }

    public Command intakeAlgaeGroundCommand() {
        return intakeCommand(() -> SuperstructureConstants.ALGAE_GROUND_INTAKING_STATE, 
            SuperstructureConstants.ALGAE_STOWED_STATE);
    }

    public Command intakeAlgaeReefLowCommand() {
        return intakeCommand(() -> SuperstructureConstants.ALGAE_LOW_REEF_INTAKING_STATE, 
            SuperstructureConstants.ALGAE_STOWED_STATE);
    }

    public Command intakeAlgaeReefHighCommand() {
        return intakeCommand(() -> SuperstructureConstants.ALGAE_HIGH_REEF_INTAKING_STATE, 
            SuperstructureConstants.ALGAE_STOWED_STATE);
    }

    public Command intakeAlgaeReefCommand(Supplier<Pose2d> targetPose) {
        Supplier<SuperstructureState> intakingState = () -> {
            if (!(targetPose == null) && (targetPose.get().equals(PathSetpoints.BLUE_REEF_AB) || targetPose.get().equals(PathSetpoints.BLUE_REEF_EF) 
                    || targetPose.get().equals(PathSetpoints.BLUE_REEF_IJ) || targetPose.get().equals(PathSetpoints.RED_REEF_AB)
                    || targetPose.get().equals(PathSetpoints.RED_REEF_EF) || targetPose.get().equals(PathSetpoints.RED_REEF_IJ)))
                return SuperstructureConstants.ALGAE_HIGH_REEF_INTAKING_STATE;
            // else if (targetPose.get().equals(PathSetpoints.REEF_CD) || targetPose.get().equals(PathSetpoints.REEF_GH) 
            //         || targetPose.get().equals(PathSetpoints.REEF_KL)) 
            //     return SuperstructureConstants.ALGAE_LOW_REEF_INTAKING_STATE;
            else return SuperstructureConstants.ALGAE_LOW_REEF_INTAKING_STATE;
        };

        return intakeCommand(intakingState, SuperstructureConstants.ALGAE_STOWED_STATE);
    }

    public Command intakeCommand(Supplier<SuperstructureState> intakingState, SuperstructureState stowedState) {
        Debouncer debouncer = new Debouncer(IntakeConstants.STALL_DEBOUNCE_TIME, DebounceType.kRising);

        return Commands.sequence(
            Commands.runOnce(() -> {
                debouncer.calculate(false);
                setState(intakingState.get());
            }, elevator, arm, wrist, intake),
            Commands.waitUntil(() -> (debouncer.calculate(Math.abs(intake.getFilteredCurrentLeft()) > IntakeConstants.STALL_CURRENT
                || Math.abs(intake.getFilteredCurrentRight()) > IntakeConstants.STALL_CURRENT)) || Robot.isSimulation()),
            Commands.runOnce(() -> led.flashAllPixels(LedConstants.BLUE, 5), led)
        )
        .finallyDo(
            () -> setState(stowedState)
        );
    }

    // Scoring commands

    public Command scoreCoralL1Command(Supplier<Boolean> isNearTargetPose, Supplier<Boolean> isAtTargetPose, 
            Supplier<Boolean> overrideNearPose, Supplier<Boolean> overrideAtPose) {
        return scoreEjectCommand(SuperstructureConstants.STAGE_L1_STATE, SuperstructureConstants.EJECT_L1_STATE,
            SuperstructureConstants.CORAL_STOWED_STATE, 
            isNearTargetPose, isAtTargetPose, overrideNearPose, overrideAtPose);
    }

    public Command scoreCoralL2Command(Supplier<Boolean> isNearTargetPose, Supplier<Boolean> isAtTargetPose, 
            Supplier<Boolean> overrideNearPose, Supplier<Boolean> overrideAtPose) {
        return scorePlaceCommand(SuperstructureConstants.TRAVEL_L2_STATE, SuperstructureConstants.STAGE_L2_STATE, 
            SuperstructureConstants.PLACE_L2_STATE, SuperstructureConstants.RETRACT_L2_STATE, SuperstructureConstants.CORAL_STOWED_STATE, 
            isNearTargetPose, isAtTargetPose, 
            overrideNearPose, overrideAtPose);
    }

    public Command scoreCoralL3Command(Supplier<Boolean> isNearTargetPose, Supplier<Boolean> isAtTargetPose, 
            Supplier<Boolean> overrideNearPose, Supplier<Boolean> overrideAtPose) {
        return scorePlaceCommand(SuperstructureConstants.TRAVEL_L3_STATE, SuperstructureConstants.STAGE_L3_STATE, 
            SuperstructureConstants.PLACE_L3_STATE, SuperstructureConstants.RETRACT_L3_STATE, SuperstructureConstants.CORAL_STOWED_STATE, 
            isNearTargetPose, isAtTargetPose, 
            overrideNearPose, overrideAtPose);
    }

    public Command scoreCoralL4Command(Supplier<Boolean> isNearTargetPose, Supplier<Boolean> isAtTargetPose, 
            Supplier<Boolean> overrideNearPose, Supplier<Boolean> overrideAtPose) {
        return scorePlaceCommand(SuperstructureConstants.TRAVEL_L4_STATE, SuperstructureConstants.STAGE_L4_STATE, 
            SuperstructureConstants.PLACE_L4_STATE, SuperstructureConstants.RETRACT_L4_STATE, SuperstructureConstants.CORAL_STOWED_STATE, 
            isNearTargetPose, isAtTargetPose, 
            overrideNearPose, overrideAtPose);
    }

    public Command scoreAlgaeProcessorCommand(Supplier<Boolean> isNearTargetPose, Supplier<Boolean> isAtTargetPose, 
            Supplier<Boolean> overrideNearPose, Supplier<Boolean> overrideAtPose) {
        return scoreEjectCommand(SuperstructureConstants.STAGE_PROCESSOR_STATE, SuperstructureConstants.EJECT_PROCESSOR_STATE,
            SuperstructureConstants.ALGAE_STOWED_STATE, 
            isNearTargetPose, isAtTargetPose, overrideNearPose, overrideAtPose);
    }

    public Command scoreAlgaeNetCommand(Supplier<Boolean> isNearTargetPose, Supplier<Boolean> isAtTargetPose, 
            Supplier<Boolean> overrideNearPose, Supplier<Boolean> overrideAtPose) {
        return scoreEjectCommand(SuperstructureConstants.STAGE_NET_STATE, SuperstructureConstants.EJECT_NET_STATE,
            SuperstructureConstants.ALGAE_STOWED_STATE, 
            isNearTargetPose, isAtTargetPose, overrideNearPose, overrideAtPose);
    }

    public Command scorePlaceCommand(SuperstructureState travelState, SuperstructureState stagingState, 
            SuperstructureState placingState, SuperstructureState retractingState, SuperstructureState stowedState, 
            Supplier<Boolean> isNearTargetPose, Supplier<Boolean> isAtTargetPose, 
            Supplier<Boolean> overrideNearPose, Supplier<Boolean> overrideAtPose) {
        return Commands.sequence(
            Commands.waitUntil(() -> (isNearTargetPose.get() || overrideNearPose.get())),
            Commands.runOnce(() -> setState(travelState),
                elevator, arm, wrist, intake),
            Commands.waitUntil(() -> ((isElevatorAtTargetPosition()) || overrideAtPose.get())),
            Commands.runOnce(() -> setState(stagingState),
                elevator, arm, wrist, intake),
            Commands.waitUntil(() -> ((isAtTargetState() && isAtTargetPose.get()) || overrideAtPose.get())),
            Commands.runOnce(() -> setState(placingState),
                elevator, arm, wrist, intake),
            Commands.waitUntil(() -> isAtTargetState()),
            Commands.runOnce(() -> {
                setState(retractingState);
                led.flashAllPixels(LedConstants.YELLOW, 5);
            },
                elevator, arm, wrist, intake, led),
            Commands.waitUntil(() -> isAtTargetState())
        )
        .finallyDo(() -> {
            setState(stowedState);
        });
    }

    public Command scoreEjectCommand(SuperstructureState stagingState, SuperstructureState ejectingState, 
            SuperstructureState stowedState, 
            Supplier<Boolean> isNearTargetPose, Supplier<Boolean> isAtTargetPose, 
            Supplier<Boolean> overrideNearPose, Supplier<Boolean> overrideAtPose) {
        return Commands.sequence(
            Commands.waitUntil(() -> (isNearTargetPose.get() || overrideNearPose.get())),
            Commands.runOnce(() -> setState(stagingState),
                elevator, arm, wrist, intake),
            Commands.waitUntil(() -> (isAtTargetState() && isAtTargetPose.get()) || overrideAtPose.get()),
            Commands.run(() -> {
                setState(ejectingState);
                led.flashAllPixels(LedConstants.YELLOW, 5);
            },
                elevator, arm, wrist, intake, led)
        )
        .finallyDo(() -> {
            setState(stowedState);
        });
    }

    // Temporary values
    public Command autoScoreCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> setState(SuperstructureConstants.STAGE_L1_STATE),
                elevator, arm, wrist, intake),
            Commands.waitUntil(() -> isAtTargetState()),
            Commands.run(() -> {
                setState(SuperstructureConstants.EJECT_L1_STATE);
                led.flashAllPixels(LedConstants.YELLOW, 5);
            },
                elevator, arm, wrist, intake, led)
        )
        .finallyDo(() -> {
            setState(SuperstructureConstants.CORAL_STOWED_STATE);
        });
    }

}
