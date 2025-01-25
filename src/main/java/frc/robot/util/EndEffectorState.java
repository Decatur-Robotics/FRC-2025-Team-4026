package frc.robot.util;

public class EndEffectorState {
    
    public double clawPosition;
    public double intakeVelocity;

    public EndEffectorState(double clawPosition, double intakeVelocity) {
        this.clawPosition = clawPosition;
        this.intakeVelocity = intakeVelocity;
    }

    public EndEffectorState copyInstance() {
        return new EndEffectorState(clawPosition, intakeVelocity);
    }

}
