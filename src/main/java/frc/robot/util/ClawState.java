package frc.robot.util;

public class ClawState {
    
    public double clawPosition;
    public double rollerVelocity;

    public ClawState(double clawPosition, double rollerVelocity) {
        this.clawPosition = clawPosition;
        this.rollerVelocity = rollerVelocity;
    }

    public ClawState copyInstance() {
        return new ClawState(clawPosition, rollerVelocity);
    }

}
