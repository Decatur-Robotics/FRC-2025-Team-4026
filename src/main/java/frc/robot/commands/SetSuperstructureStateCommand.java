package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.util.SuperstructureState;

public class SetSuperstructureStateCommand extends InstantCommand {

    private SuperstructureSubsystem superstructure;

    private SuperstructureState state;

    public SetSuperstructureStateCommand(SuperstructureSubsystem superstructure, SuperstructureState state) {
        this.superstructure = superstructure;
        this.state = state;

        addRequirements(superstructure);
    }

    @Override
    public void initialize() {
        superstructure.setState(state);
    }

}
