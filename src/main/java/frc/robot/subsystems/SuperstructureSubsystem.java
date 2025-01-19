package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SuperstructureStates;
import frc.robot.util.SuperstructureState;

public class SuperstructureSubsystem extends SubsystemBase {
    
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private WristSubsystem wrist;

    private SuperstructureState goalState;

    public SuperstructureSubsystem(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist) {
        this.elevator = elevator;
        this.arm = arm;
        this.wrist = wrist;

        goalState = SuperstructureStates.CORAL_STOWED_STATE;
    }

    @Override
    public void periodic() {
        
    }

    public void setState(SuperstructureState goalState) {
        this.goalState = goalState;

        elevator.setPosition(goalState.elevatorPosition);
        arm.setPosition(goalState.armPosition);
        wrist.setPosition(goalState.wristPosition);
    }

    public SuperstructureState getActualState() {
        return new SuperstructureState(elevator.getPosition(), arm.getPosition(), wrist.getPosition());
    }

    public SuperstructureState getGoalState() {
        return goalState;
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
        return goalState.elevatorPosition;
    }

    public double getGoalArmPosition() {
        return goalState.armPosition;
    }

    public double getGoalWristPosition() {
        return goalState.wristPosition;
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
