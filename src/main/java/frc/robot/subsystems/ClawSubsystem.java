package frc.robot.subsystems;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.RobotContainer;
import frc.robot.constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    
    private TalonFX motor;

    private double current;

    private TorqueCurrentFOC controlRequest;

    private LinearFilter velocityFilter;
    private double filteredVelocity;

    private Debouncer slamDebouncer;

    public ClawSubsystem() {
        motor = new TalonFX(Ports.CLAW_MOTOR); 

        motor.getConfigurator().apply(ClawConstants.MOTOR_CONFIG);

        motor.optimizeBusUtilization();

        current = ClawConstants.REDUCED_CLOSED_CURRENT;

        controlRequest = new TorqueCurrentFOC(current);
        motor.setControl(controlRequest.withOutput(current));

        velocityFilter = LinearFilter.movingAverage(10);

        slamDebouncer = new Debouncer(ClawConstants.SLAM_DEBOUNCE_TIME);

        configureShuffleboard();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = RobotContainer.getShuffleboardTab();

        tab.addDouble("Target Claw Current", () -> current);
        tab.addDouble("Actual Claw Current", () -> getCurrent());
        tab.addDouble("Filtered Claw Velocity", () -> filteredVelocity);
        tab.addBoolean("Is Claw Slammed", () -> isSlammed());
    }

    @Override
    public void periodic() {
        if (motor.hasResetOccurred()) {
			motor.optimizeBusUtilization();
		}

        filteredVelocity = velocityFilter.calculate(motor.getVelocity().getValueAsDouble());

        if (isSlammed() && (current == ClawConstants.CLOSED_CURRENT)) {
            motor.setControl(controlRequest.withOutput(ClawConstants.REDUCED_CLOSED_CURRENT));
        }
        else if (isSlammed() && (current == ClawConstants.OPEN_CURRENT)) {
            motor.setControl(controlRequest.withOutput(ClawConstants.REDUCED_OPEN_CURRENT));
        }
    }

    public void setCurrent(double current) {
        this.current = current;
        motor.setControl(controlRequest.withOutput(current));
    }

    public double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public boolean isSlammed() {
        return slamDebouncer.calculate(filteredVelocity < ClawConstants.MAX_SLAMMED_VELOCITY);
    }

    public Command setCurrentCommand(double current) {
        return Commands.startEnd(() -> setCurrent(current), 
            () -> setCurrent(0));
    }

}
