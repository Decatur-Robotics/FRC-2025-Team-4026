package frc.robot.subsystems;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import frc.robot.constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

    private TalonFX motor;

    private double current;

    private TorqueCurrentFOC controlRequest;

    private LinearFilter velocityFilter;
    private double filteredVelocity;

    private Debouncer slamDebouncer;

    public WristSubsystem() {
        motor = new TalonFX(Ports.WRIST_MOTOR);

        motor.getConfigurator().apply(WristConstants.MOTOR_CONFIG);

        motor.optimizeBusUtilization();
        motor.getVelocity().setUpdateFrequency(20);
        motor.getRotorVelocity().setUpdateFrequency(20);
        motor.getStatorCurrent().setUpdateFrequency(20);

        current = 0; 

        controlRequest = new TorqueCurrentFOC(current);
        // motor.setControl(controlRequest.withOutput(current));

        velocityFilter = LinearFilter.movingAverage(10);

        slamDebouncer = new Debouncer(WristConstants.SLAM_DEBOUNCE_TIME);

        configureShuffleboard();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.SHUFFLEBOARD_SUPERSTRUCTURE_TAB);

        tab.addDouble("Target Wrist Current", () -> current);
        tab.addDouble("Actual Wrist Current", () -> getCurrent());
        tab.addDouble("Filtered Wrist Velocity", () -> filteredVelocity);
        tab.addBoolean("Is Wrist Slammed", () -> isSlammed());
    }

    @Override
    public void periodic() {
        if(motor.hasResetOccurred()) {
            motor.optimizeBusUtilization();
        }

        filteredVelocity = velocityFilter.calculate(motor.getVelocity().getValueAsDouble());

        if (isSlammed() && (current == WristConstants.PARALLEL_CURRENT)) {
            setCurrent(WristConstants.REDUCED_PARALLEL_CURRENT);
        }
        else if (isSlammed() && (current == WristConstants.PERPENDICULAR_CURRENT)) {
            setCurrent(WristConstants.REDUCED_PERPENDICULAR_CURRENT);
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
        return slamDebouncer.calculate(filteredVelocity < WristConstants.MAX_SLAMMED_VELOCITY);
    }

    public Command setCurrentCommand(double current) {
        return Commands.runEnd(() -> setCurrent(current), 
            () -> setCurrent(0),
            this);
    }

}
