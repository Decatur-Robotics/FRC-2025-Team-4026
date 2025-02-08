package frc.robot.util;

public class SuperstructureState {
    
    public double elevatorPosition;
    public double armPosition;
    public double wristPosition;
    public double clawPosition;

    public SuperstructureState(double elevatorPosition, double armPosition, double wristPosition, double clawPosition) {
        this.elevatorPosition = elevatorPosition;
        this.armPosition = armPosition;
        this.wristPosition = wristPosition;
        this.clawPosition = clawPosition;
    }

    public SuperstructureState copyInstance() {
        return new SuperstructureState(elevatorPosition, armPosition, wristPosition, clawPosition);
    }
    
}
