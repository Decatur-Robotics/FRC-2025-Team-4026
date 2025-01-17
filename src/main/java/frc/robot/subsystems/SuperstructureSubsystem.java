package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SuperstructureState;

public class SuperstructureSubsystem extends SubsystemBase{
    
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private WristSubsystem wrist;

    private SuperstructureState state;

    public SuperstructureSubsystem() {
        elevator = new ElevatorSubsystem();
        arm = new ArmSubsystem();
        wrist = new WristSubsystem();
    }

    public void setState(SuperstructureState state) {
        this.state = state;

        elevator.setPosition(state.elevatorPosition);
        arm.setPosition(state.armPosition);
        wrist.setPosition(state.wristPosition);
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

    public double getElevatorPosition() {
        return elevator.getPosition();
    }

    public double getArmPosition() {
        return arm.getPosition();
    }

    public double getWristPosition() {
        return wrist.getPosition();
    }

    public SuperstructureState getState() {
        state = new SuperstructureState(elevator.getPosition(), arm.getPosition(), wrist.getPosition());
        return state;
    }

}
