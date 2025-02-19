package frc.robot.util;

public class SuperstructureState {
    
    public double elevatorPosition;
    public double armPosition;
    public double wristCurrent;
    public double clawCurrent;

    public SuperstructureState(double elevatorPosition, double armPosition, double wristCurrent, double clawCurrent) {
        this.elevatorPosition = elevatorPosition;
        this.armPosition = armPosition;
        this.wristCurrent = wristCurrent;
        this.clawCurrent = clawCurrent;
    }

    public SuperstructureState copyInstance() {
        return new SuperstructureState(elevatorPosition, armPosition, wristCurrent, clawCurrent);
    }
    
}
