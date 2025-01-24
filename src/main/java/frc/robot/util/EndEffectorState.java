package frc.robot.util;

public class EndEffectorState {
    
    public double clawPosition;
    public double rollerVelocity;

    public EndEffectorState(double clawPosition, double rollerVelocity) {
        this.clawPosition = clawPosition;
        this.rollerVelocity = rollerVelocity;
    }

    public EndEffectorState copyInstance() {
        return new EndEffectorState(clawPosition, rollerVelocity);
    }

}
