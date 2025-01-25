package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureConstants;
import frc.robot.util.SuperstructureState;

public class SuperstructureSubsystem extends SubsystemBase {
    
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private WristSubsystem wrist;

    private SuperstructureState goalTargetState;
    private SuperstructureState targetState;

    public SuperstructureSubsystem(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;

        goalTargetState = SuperstructureConstants.CORAL_STOWED_STATE.copyInstance();
        targetState = SuperstructureConstants.CORAL_STOWED_STATE.copyInstance();
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
                    goalTargetState.wristPosition);
        }
        else if (getActualArmPosition() < SuperstructureConstants.ARM_MINIMUM_STOWED_POSITION) {
            targetState = new SuperstructureState(
                    Math.max(SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION, goalTargetState.elevatorPosition), 
                    goalTargetState.armPosition, goalTargetState.wristPosition);
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

    public SuperstructureState getActualState() {
        return new SuperstructureState(getActualElevatorPosition(), getActualArmPosition(), getActualWristPosition());
    }

    public boolean isAtTargetState() {
        return isElevatorAtTargetPosition() &&
                isArmAtTargetPosition() &&
                isWristAtTargetPosition();
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

    public SuperstructureState getGoalTargetState() {
        return goalTargetState;
    }

    public double getActualElevatorPosition() {
        return elevator.getPosition();
    }

    public double getActualArmPosition() {
        return arm.getPosition();
    }

    public double getActualWristPosition() {
        return wrist.getPosition();
    }

    public double getGoalElevatorPosition() {
        return goalTargetState.elevatorPosition;
    }

    public double getGoalArmPosition() {
        return goalTargetState.armPosition;
    }

    public double getGoalWristPosition() {
        return goalTargetState.wristPosition;
    }

    // Directly set subsystem positions

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

}
