package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

    private ClawSubsystem claw;
    private IntakeSubsystem intake;

    public EndEffectorSubsystem(ClawSubsystem claw, IntakeSubsystem intake) {
        this.claw = claw;
        this.intake = intake;
    }
    
}
