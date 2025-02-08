package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
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
                wrist.setPosition(targetState.wristPosition);
            }
        }
    }

    private void calculateCollisionAvoidanceState() {
        if (getActualElevatorPosition() < SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION) {
            targetState = new SuperstructureState(goalTargetState.elevatorPosition, 
                    Math.max(SuperstructureConstants.ARM_MINIMUM_STOWED_POSITION, goalTargetState.armPosition), 
                    goalTargetState.wristPosition, goalTargetState.clawPosition);
        }
        else if (getActualArmPosition() < SuperstructureConstants.ARM_MINIMUM_STOWED_POSITION) {
            targetState = new SuperstructureState(
                    Math.max(SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION, goalTargetState.elevatorPosition), 
                    goalTargetState.armPosition, goalTargetState.wristPosition, goalTargetState.clawPosition);
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
        wrist.setPosition(targetState.wristPosition);
    }

    // Is at targets

    public boolean isAtTargetState() {
        return isElevatorAtTargetPosition() &&
                isArmAtTargetPosition() &&
                isWristAtTargetPosition() &&
                isClawAtTargetPosition();
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
        if (Math.abs(getActualWristPosition() - targetState.wristPosition) > SuperstructureConstants.WRIST_ERROR_MARGIN) {
            return false;
        }

        return true;
    }

    public boolean isClawAtTargetPosition() {
        if (Math.abs(getActualClawPosition() - targetState.clawPosition) > SuperstructureConstants.CLAW_ERROR_MARGIN) {
            return false;
        }

        return true;
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
        return new SuperstructureState(getActualElevatorPosition(), getActualArmPosition(), getActualWristPosition(), getActualClawPosition());
    }

    public double getActualElevatorPosition() {
        return elevator.getPosition();
    }

    public double getActualArmPosition() {
        return arm.getOffsetTalonPosition();
    }

    public double getActualWristPosition() {
        return wrist.getOffsetTalonPosition();
    }

    public double getActualClawPosition() {
        return claw.getOffsetTalonPosition();
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

    public void setWristPosition(double position) {
        targetState.wristPosition = position;
        
        wrist.setPosition(position);
    }

    public void setClawPosition(double position) {
        targetState.clawPosition = position;
        
        claw.setPosition(position);
    }

    public void setIntakeVelocity(double velocity) {
        targetIntakeVelocity = velocity;
        
        claw.setPosition(velocity);
    }

    // Get other subsystem information

    public double getFilteredIntakeCurrent() {
        return intake.getFilteredCurrent();
    }

    // Intaking commands

    public Command intakeCoralGroundCommand() {
        Debouncer debouncer = new Debouncer(IntakeConstants.STALL_DEBOUNCE_TIME, DebounceType.kRising);

        return runOnce(
            () -> {
                debouncer.calculate(false);
            })
            .andThen(
                run(() -> setState(SuperstructureConstants.CORAL_GROUND_INTAKING_STATE))
            )
            .alongWith(
                run(() -> setIntakeVelocity(IntakeConstants.INTAKE_VELOCITY))
            )
            .until(
                () -> debouncer.calculate(getFilteredIntakeCurrent() > IntakeConstants.STALL_CURRENT)
            )
            .finallyDo(
                () -> run(() -> setState(SuperstructureConstants.CORAL_STOWED_STATE))
            );
    }

}
