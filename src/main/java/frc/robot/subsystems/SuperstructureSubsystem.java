package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.PathSetpoints;
import frc.robot.constants.SuperstructureConstants;
import frc.robot.util.SuperstructureState;

public class SuperstructureSubsystem extends SubsystemBase {
    
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private WristSubsystem wrist;
    private ClawSubsystem claw;
    private IntakeSubsystem intake;

    private SuperstructureState goalTargetState;
    private SuperstructureState targetState;

    public SuperstructureSubsystem(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist,
            ClawSubsystem claw, IntakeSubsystem intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.intake = intake;

        goalTargetState = SuperstructureConstants.CORAL_STOWED_STATE.copyInstance();
        targetState = SuperstructureConstants.CORAL_STOWED_STATE.copyInstance();
    }

    @Override
    public void periodic() {
        if (!targetState.equals(goalTargetState)) {
            setState(goalTargetState);
        }
    }

    private void calculateCollisionAvoidanceState() {
        if (getActualElevatorPosition() < SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION) {
            targetState = new SuperstructureState(goalTargetState.elevatorPosition, 
                    Math.max(SuperstructureConstants.ARM_MINIMUM_STOWED_POSITION, goalTargetState.armPosition), 
                    goalTargetState.wristCurrent, goalTargetState.clawCurrent, goalTargetState.intakeVelocity);
        }
        else if (getActualArmPosition() < SuperstructureConstants.ARM_MINIMUM_STOWED_POSITION) {
            targetState = new SuperstructureState(
                    Math.max(SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION, goalTargetState.elevatorPosition), 
                    goalTargetState.armPosition, goalTargetState.wristCurrent, goalTargetState.clawCurrent, goalTargetState.intakeVelocity);
        }
        else {
            targetState = goalTargetState.copyInstance();
        }
    }

    public void setState(SuperstructureState goalTargetState) {
        this.goalTargetState = goalTargetState.copyInstance();

        SuperstructureState oldTargetState = targetState.copyInstance();
        
        calculateCollisionAvoidanceState();

        if (!targetState.equals(oldTargetState)) {
            elevator.setPosition(targetState.elevatorPosition);
            arm.setPosition(targetState.armPosition);
            wrist.setCurrent(targetState.wristCurrent);
            claw.setCurrent(targetState.clawCurrent);
            intake.setVelocity(targetState.intakeVelocity);
        }
    }

    // Is at targets

    public boolean isAtGoalTargetState() {
        return isElevatorAtGoalTargetPosition() &&
                isArmAtGoalTargetPosition() &&
                isWristAtGoalTargetPosition() &&
                isClawAtGoalTargetPosition();
    }

    public boolean isElevatorAtGoalTargetPosition() {
        if (Math.abs(getActualElevatorPosition() - targetState.elevatorPosition) > SuperstructureConstants.ELEVATOR_ERROR_MARGIN) {
            return false;
        }

        return true;
    }

    public boolean isArmAtGoalTargetPosition() {
        if (Math.abs(getActualArmPosition() - targetState.armPosition) > SuperstructureConstants.ARM_ERROR_MARGIN) {
            return false;
        }

        return true;
    }

    public boolean isWristAtGoalTargetPosition() {
        return claw.isSlammed();
    }

    public boolean isClawAtGoalTargetPosition() {
        return claw.isSlammed();
    }

    // Get goal states

    public SuperstructureState getGoalState() {
        return goalTargetState;
    }

    // Get actual states

    public SuperstructureState getActualState() {
        return new SuperstructureState(getActualElevatorPosition(), getActualArmPosition(), 
            getActualWristCurrent(), getActualClawCurrent(), getActualIntakeVelocity());
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

    public double getActualClawCurrent() {
        return claw.getCurrent();
    }

    public double getActualIntakeVelocity() {
        return intake.getVelocity();
    }

    // Directly set subsystem targets

    public void setElevatorPosition(double position) {
        goalTargetState.elevatorPosition = position;
        targetState.elevatorPosition = position;
        
        elevator.setPosition(position);
    }

    public void setArmPosition(double position) {
        goalTargetState.armPosition = position;
        targetState.armPosition = position;
        
        arm.setPosition(position);
    }

    public void setWristCurrent(double current) {
        goalTargetState.wristCurrent = current;
        targetState.wristCurrent = current;
        
        wrist.setCurrent(current);
    }

    public void setClawCurrent(double current) {
        goalTargetState.clawCurrent = current;
        targetState.clawCurrent = current;
        
        claw.setCurrent(current);
    }

    public void setIntakeVelocity(double velocity) {
        goalTargetState.intakeVelocity = velocity;
        targetState.intakeVelocity = velocity;
        
        intake.setVelocity(velocity);
    }

    // Reset offsets commands

    public Command zeroSuperstructureCommand() {
        return Commands.sequence(Commands.runOnce(() -> setArmPosition(SuperstructureConstants.CORAL_STOWED_STATE.armPosition)),
            Commands.waitUntil(() -> isArmAtGoalTargetPosition()))
            .finallyDo(() -> elevator.zeroCommand());
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

    public Command intakeAlgaeReefCommand(Supplier<Pose2d> targetPose) {
        Supplier<SuperstructureState> intakingState = () -> {
            if (targetPose.get().equals(PathSetpoints.REEF_AB) || targetPose.get().equals(PathSetpoints.REEF_EF) 
                    || targetPose.get().equals(PathSetpoints.REEF_IJ)) 
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
            }),
            Commands.waitUntil(() -> debouncer.calculate(intake.getFilteredCurrent() > IntakeConstants.STALL_CURRENT))
        )
        .finallyDo(
            () -> {
                setState(stowedState);
            }
        );
    }

    // Scoring commands

    public Command scoreCoralL1Command(Supplier<Boolean> isAtTargetPose, Supplier<Boolean> overrideLineUp) {
        return scoreCommand(SuperstructureConstants.MOVE_TO_L1_STATE, SuperstructureConstants.SCORE_L1_STATE,
            IntakeConstants.L1_EJECT_VELOCITY, SuperstructureConstants.CORAL_SCORE_TO_STOW_DELAY, SuperstructureConstants.CORAL_STOWED_STATE,
            isAtTargetPose, overrideLineUp);
    }

    public Command scoreCoralL2Command(Supplier<Boolean> isAtTargetPose, Supplier<Boolean> overrideLineUp) {
        return scoreCommand(SuperstructureConstants.MOVE_TO_L2_STATE, SuperstructureConstants.SCORE_L2_STATE,
            IntakeConstants.REST_VELOCITY, SuperstructureConstants.CORAL_SCORE_TO_STOW_DELAY, SuperstructureConstants.CORAL_STOWED_STATE,
            isAtTargetPose, overrideLineUp);
    }

    public Command scoreCoralL3Command(Supplier<Boolean> isAtTargetPose, Supplier<Boolean> overrideLineUp) {
        return scoreCommand(SuperstructureConstants.MOVE_TO_L3_STATE, SuperstructureConstants.SCORE_L3_STATE,
            IntakeConstants.REST_VELOCITY, SuperstructureConstants.CORAL_SCORE_TO_STOW_DELAY, SuperstructureConstants.CORAL_STOWED_STATE,
            isAtTargetPose, overrideLineUp);
    }

    public Command scoreCoralL4Command(Supplier<Boolean> isAtTargetPose, Supplier<Boolean> overrideLineUp) {
        return scoreCommand(SuperstructureConstants.MOVE_TO_L4_STATE, SuperstructureConstants.SCORE_L4_STATE,
            IntakeConstants.REST_VELOCITY, SuperstructureConstants.CORAL_SCORE_TO_STOW_DELAY, SuperstructureConstants.CORAL_STOWED_STATE,
            isAtTargetPose, overrideLineUp);
    }

    public Command scoreAlgaeProcessorCommand(Supplier<Boolean> isAtTargetPose, Supplier<Boolean> overrideLineUp) {
        return scoreCommand(SuperstructureConstants.MOVE_TO_PROCESSOR_STATE, SuperstructureConstants.SCORE_PROCESSOR_STATE,
            IntakeConstants.PROCESSOR_EJECT_VELOCITY, SuperstructureConstants.ALGAE_SCORE_TO_STOW_DELAY, SuperstructureConstants.ALGAE_STOWED_STATE,
            isAtTargetPose, overrideLineUp);
    }

    public Command scoreAlgaeNetCommand(Supplier<Boolean> isAtTargetPose, Supplier<Boolean> overrideLineUp) {
        return scoreCommand(SuperstructureConstants.MOVE_TO_NET_STATE, SuperstructureConstants.SCORE_NET_STATE,
            IntakeConstants.NET_EJECT_VELOCITY, SuperstructureConstants.ALGAE_SCORE_TO_STOW_DELAY, SuperstructureConstants.ALGAE_STOWED_STATE,
            isAtTargetPose, overrideLineUp);
    }

    public Command scoreCommand(SuperstructureState stagingState, SuperstructureState scoringState, 
            double ejectVelocity, double scoreToStowDelay, SuperstructureState stowedState, 
            Supplier<Boolean> isAtTargetPose, Supplier<Boolean> overrideLineUp) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                setState(stagingState);
            }),
            Commands.waitUntil(() -> (isAtGoalTargetState() && isAtTargetPose.get()) || overrideLineUp.get()),
            Commands.runOnce(() -> {
                setState(scoringState);
            }),
            Commands.waitSeconds(scoreToStowDelay)
        )
        .finallyDo(() -> {
            setState(stowedState);
        });
    }

}
