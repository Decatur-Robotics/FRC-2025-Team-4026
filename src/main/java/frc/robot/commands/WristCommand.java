package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends Command {
    private double position;
    private WristSubsystem wrist;
    public WristCommand(){
        this.wrist = wrist;
        this.position = position;

        this.addRequirements(wrist);
    }

    public void initialize(){
        wrist.setPosition(position);
    }
}
