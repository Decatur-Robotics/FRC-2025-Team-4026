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

    private double targetIntakeVelocity;

    public SuperstructureSubsystem(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist,
            ClawSubsystem claw, IntakeSubsystem intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;
        this.claw = claw;
        this.intake = intake;

        goalTargetState = SuperstructureConstants.CORAL_STOWED_STATE.copyInstance();
        targetState = SuperstructureConstants.CORAL_STOWED_STATE.copyInstance();

        targetIntakeVelocity = IntakeConstants.REST_VELOCITY;
    }

    @Override
    public void periodic() {
        if (!targetState.equals(goalTargetState)) {
            SuperstructureState oldTargetState = targetState.copyInstance();

            calculateCollisionAvoidanceState();

            if (!targetState.equals(oldTargetState)) {
                elevator.setPosition(targetState.elevatorPosition);
                arm.setPosition(targetState.armPosition);
                wrist.setCurrent(targetState.wristCurrent);
                claw.setCurrent(targetState.clawCurrent);
            }
        }
    }

    private void calculateCollisionAvoidanceState() {
        if (getActualElevatorPosition() < SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION) {
            targetState = new SuperstructureState(goalTargetState.elevatorPosition, 
                    Math.max(SuperstructureConstants.ARM_MINIMUM_STOWED_POSITION, goalTargetState.armPosition), 
                    goalTargetState.wristCurrent, goalTargetState.clawCurrent);
        }
        else if (getActualArmPosition() < SuperstructureConstants.ARM_MINIMUM_STOWED_POSITION) {
            targetState = new SuperstructureState(
                    Math.max(SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION, goalTargetState.elevatorPosition), 
                    goalTargetState.armPosition, goalTargetState.wristCurrent, goalTargetState.clawCurrent);
        }
        else {
            targetState = goalTargetState.copyInstance();
        }
    }

    public void setState(SuperstructureState goalTargetState) {
        this.goalTargetState = goalTargetState.copyInstance();
        
        calculateCollisionAvoidanceState();

        elevator.setPosition(targetState.elevatorPosition);
        arm.setPosition(targetState.armPosition);
        wrist.setCurrent(targetState.wristCurrent);
        claw.setCurrent(targetState.clawCurrent);
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

    public double getTargetIntakeVelocity() {
        return targetIntakeVelocity;
    }

    // Get actual states

    public SuperstructureState getActualState() {
        return new SuperstructureState(getActualElevatorPosition(), getActualArmPosition(), getActualWristCurrent(), getActualClawCurrent());
    }

    public double getActualElevatorPosition() {
        return elevator.getPosition();
    }

    public double getActualArmPosition() {
        return arm.getOffsetTalonPosition();
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
        targetState.elevatorPosition = position;
        
        elevator.setPosition(position);
    }

    public void setArmPosition(double position) {
        targetState.armPosition = position;
        
        arm.setPosition(position);
    }

    public void setWristCurrent(double current) {
        targetState.wristCurrent = current;
        
        wrist.setCurrent(current);
    }

    public void setClawCurrent(double current) {
        targetState.clawCurrent = current;
        
        claw.setCurrent(current);
    }

    public void setIntakeVelocity(double velocity) {
        targetIntakeVelocity = velocity;
        
        claw.setCurrent(velocity);
    }

    // Get other subsystem information

    public double getFilteredIntakeCurrent() {
        return intake.getFilteredCurrent();
    }

    // Reset offsets commands

    public Command zeroSuperstructureCommand() {
        return Commands.sequence(Commands.runOnce(() -> setArmPosition(SuperstructureConstants.CORAL_STOWED_STATE.armPosition)),
            Commands.waitUntil(() -> isArmAtGoalTargetPosition()))
            .finallyDo(() -> elevator.zeroCommand());
    }

    // Intaking commands

    public Command intakeCoralGroundCommand() {
        Debouncer debouncer = new Debouncer(IntakeConstants.STALL_DEBOUNCE_TIME, DebounceType.kRising);

        return Commands.sequence(
            Commands.runOnce(() -> {
                debouncer.calculate(false);
                setState(SuperstructureConstants.CORAL_GROUND_INTAKING_STATE);
                setIntakeVelocity(IntakeConstants.CORAL_INTAKE_VELOCITY);
            }),
            Commands.waitUntil(() -> debouncer.calculate(getFilteredIntakeCurrent() > IntakeConstants.STALL_CURRENT))
        )
        .finallyDo(
            () -> {
                setState(SuperstructureConstants.CORAL_STOWED_STATE);
                setIntakeVelocity(IntakeConstants.REST_VELOCITY);
            }
        );
    }

    public Command intakeCoralHumanPlayerCommand() {
        Debouncer debouncer = new Debouncer(IntakeConstants.STALL_DEBOUNCE_TIME, DebounceType.kRising);

        return Commands.sequence(
            Commands.runOnce(() -> {
                debouncer.calculate(false);
                setState(SuperstructureConstants.CORAL_HUMAN_PLAYER_INTAKING_STATE);
                setIntakeVelocity(IntakeConstants.CORAL_INTAKE_VELOCITY);
            }),
            Commands.waitUntil(() -> debouncer.calculate(getFilteredIntakeCurrent() > IntakeConstants.STALL_CURRENT))
        )
        .finallyDo(
            () -> {
                setState(SuperstructureConstants.CORAL_STOWED_STATE);
                setIntakeVelocity(IntakeConstants.REST_VELOCITY);
            }
        );
    }

    public Command intakeAlgaeGroundCommand() {
        Debouncer debouncer = new Debouncer(IntakeConstants.STALL_DEBOUNCE_TIME, DebounceType.kRising);

        return Commands.sequence(
            Commands.runOnce(() -> {
                debouncer.calculate(false);
                setState(SuperstructureConstants.ALGAE_GROUND_INTAKING_STATE);
                setIntakeVelocity(IntakeConstants.ALGAE_INTAKE_VELOCITY);
            }),
            Commands.waitUntil(() -> debouncer.calculate(getFilteredIntakeCurrent() > IntakeConstants.STALL_CURRENT))
        )
        .finallyDo(
            () -> {
                setState(SuperstructureConstants.ALGAE_STOWED_STATE);
                setIntakeVelocity(IntakeConstants.REST_VELOCITY);
            }
        );
    }

    public Command intakeAlgaeReefCommand(Supplier<Pose2d> targetPose) {
        Debouncer debouncer = new Debouncer(IntakeConstants.STALL_DEBOUNCE_TIME, DebounceType.kRising);

        Supplier<SuperstructureState> intakingState = () -> {
            if (targetPose.get().equals(PathSetpoints.REEF_AB) || targetPose.get().equals(PathSetpoints.REEF_EF) 
                    || targetPose.get().equals(PathSetpoints.REEF_IJ)) 
                return SuperstructureConstants.ALGAE_HIGH_REEF_INTAKING_STATE;
            // else if (targetPose.get().equals(PathSetpoints.REEF_CD) || targetPose.get().equals(PathSetpoints.REEF_GH) 
            //         || targetPose.get().equals(PathSetpoints.REEF_KL)) 
            //     return SuperstructureConstants.ALGAE_LOW_REEF_INTAKING_STATE;
            else return SuperstructureConstants.ALGAE_LOW_REEF_INTAKING_STATE;
        };

        return Commands.sequence(
            Commands.runOnce(() -> {
                debouncer.calculate(false);
                setState(intakingState.get());
                setIntakeVelocity(IntakeConstants.ALGAE_INTAKE_VELOCITY);
            }),
            Commands.waitUntil(() -> debouncer.calculate(getFilteredIntakeCurrent() > IntakeConstants.STALL_CURRENT))
        )
        .finallyDo(
            () -> {
                setState(SuperstructureConstants.ALGAE_STOWED_STATE);
                setIntakeVelocity(IntakeConstants.REST_VELOCITY);
            }
        );
    }

    public Command intakeAlgaeHighReefCommand() {
        Debouncer debouncer = new Debouncer(IntakeConstants.STALL_DEBOUNCE_TIME, DebounceType.kRising);

        return Commands.sequence(
            Commands.runOnce(() -> {
                debouncer.calculate(false);
                setState(SuperstructureConstants.ALGAE_HIGH_REEF_INTAKING_STATE);
                setIntakeVelocity(IntakeConstants.ALGAE_INTAKE_VELOCITY);
            }),
            Commands.waitUntil(() -> debouncer.calculate(getFilteredIntakeCurrent() > IntakeConstants.STALL_CURRENT))
        )
        .finallyDo(
            () -> {
                setState(SuperstructureConstants.ALGAE_STOWED_STATE);
                setIntakeVelocity(IntakeConstants.REST_VELOCITY);
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
            double ejectVelocity, double scoreToStowDelay, SuperstructureState stowState, 
            Supplier<Boolean> isAtTargetPose, Supplier<Boolean> overrideLineUp) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                setState(stagingState);
            }),
            Commands.waitUntil(() -> (isAtGoalTargetState() && isAtTargetPose.get()) || overrideLineUp.get()),
            Commands.runOnce(() -> {
                setState(scoringState);
                setIntakeVelocity(ejectVelocity);
            }),
            Commands.waitSeconds(scoreToStowDelay)
        )
        .finallyDo(() -> {
            setState(stowState);
            setIntakeVelocity(IntakeConstants.REST_VELOCITY);
        });
    }

}
