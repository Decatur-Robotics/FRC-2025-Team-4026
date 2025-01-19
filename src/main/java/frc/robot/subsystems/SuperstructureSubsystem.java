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

            if (goalTargetState.armPosition < SuperstructureConstants.ARM_MINIMUM_STOWABLE_POSITION
                    && getActualElevatorPosition() < SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION) {
                targetState = new SuperstructureState(goalTargetState.elevatorPosition, 
                        SuperstructureConstants.ARM_MINIMUM_STOWABLE_POSITION, 
                        goalTargetState.wristPosition);
            }
            else if (goalTargetState.elevatorPosition < SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION
                    && getActualArmPosition() < SuperstructureConstants.ARM_MINIMUM_STOWABLE_POSITION) {
                targetState = new SuperstructureState(SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION, 
                        goalTargetState.armPosition, goalTargetState.wristPosition);
            }
            else {
                targetState = goalTargetState.copyInstance();
            }

            if (!targetState.equals(oldTargetState)) {
                elevator.setPosition(targetState.elevatorPosition);
                arm.setPosition(targetState.armPosition);
                wrist.setPosition(targetState.wristPosition);
            }
        }
    }

    public void setState(final SuperstructureState goalTargetState) {
        this.goalTargetState = goalTargetState.copyInstance();

        if (goalTargetState.armPosition < SuperstructureConstants.ARM_MINIMUM_STOWABLE_POSITION
                && getActualElevatorPosition() < SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION) {
            targetState = new SuperstructureState(goalTargetState.elevatorPosition, 
                    SuperstructureConstants.ARM_MINIMUM_STOWABLE_POSITION, 
                    goalTargetState.wristPosition);
        }
        if (goalTargetState.elevatorPosition < SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION
                && getActualArmPosition() < SuperstructureConstants.ARM_MINIMUM_STOWABLE_POSITION) {
            targetState = new SuperstructureState(SuperstructureConstants.ELEVATOR_MINIMUM_UNSTOWED_POSITION, 
                    goalTargetState.armPosition, goalTargetState.wristPosition);
        }
        else {
            targetState = goalTargetState.copyInstance();
        }

        elevator.setPosition(targetState.elevatorPosition);
        arm.setPosition(targetState.armPosition);
        wrist.setPosition(targetState.wristPosition);
    }

    public SuperstructureState getActualState() {
        return new SuperstructureState(elevator.getPosition(), arm.getPosition(), wrist.getPosition());
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
        elevator.setPosition(position);
    }

    public void setArmPosition(double position) {
        arm.setPosition(position);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

}
