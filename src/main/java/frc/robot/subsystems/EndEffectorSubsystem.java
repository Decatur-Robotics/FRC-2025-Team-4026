package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.EndEffectorState;

public class EndEffectorSubsystem extends SubsystemBase {

    private ClawSubsystem claw;
    private IntakeSubsystem intake;

    private EndEffectorState targetState;

    public EndEffectorSubsystem(ClawSubsystem claw, IntakeSubsystem intake) {
        this.claw = claw;
        this.intake = intake;
    }

    public void setState(EndEffectorState targetState) {
        this.targetState = targetState.copyInstance();

        claw.setPosition(targetState.clawPosition);
        intake.setVelocity(targetState.intakeVelocity);
    }

    public EndEffectorState getActualState() {
        return new EndEffectorState(getActualClawPosition(), 
                getActualIntakeVelocity());
    }

    public EndEffectorState getTargetState() {
        return targetState;
    }

    public double getActualClawPosition() {
        return claw.getOffsetTalonPosition();
    }

    public double getActualIntakeVelocity() {
        return intake.getVelocity();
    }

    public double getTargetClawPosition() {
        return targetState.clawPosition;
    }

    public double getTargetIntakeVelocity() {
        return targetState.intakeVelocity;
    }

    public void setClawPosition(double position) {
        targetState.clawPosition = position;
        
        claw.setPosition(position);
    }

    public void setIntakeVelocity(double velocity) {
        targetState.intakeVelocity = velocity;
        
        claw.setPosition(velocity);
    }
    
}
