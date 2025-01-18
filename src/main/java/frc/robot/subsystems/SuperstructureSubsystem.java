package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SuperstructureState;

public class SuperstructureSubsystem extends SubsystemBase {

    private static SuperstructureSubsystem instance;
    
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private WristSubsystem wrist;

    private SuperstructureState goalState;

    public SuperstructureSubsystem() {
        elevator = ElevatorSubsystem.getInstance();
        arm = ArmSubsystem.getInstance();
        wrist = WristSubsystem.getInstance();
    }

    public static SuperstructureSubsystem getInstance() {
        if (instance == null) {
            instance = new SuperstructureSubsystem();
        }

        return instance;
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

    public void setElevatorPosition(double position) {
        elevator.setPosition(position);
    }

    public void setArmPosition(double position) {
        arm.setPosition(position);
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
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

}
