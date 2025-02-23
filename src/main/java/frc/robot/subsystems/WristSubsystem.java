package frc.robot.subsystems;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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

        current = WristConstants.PARALLEL_CURRENT; 

        controlRequest = new TorqueCurrentFOC(current);
        motor.setControl(controlRequest.withOutput(current));

        velocityFilter = LinearFilter.movingAverage(10);

        slamDebouncer = new Debouncer(WristConstants.SLAM_DEBOUNCE_TIME);

        configureShuffleboard();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = RobotContainer.getShuffleboardTab();

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
            motor.setControl(controlRequest.withOutput(WristConstants.REDUCED_PARALLEL_CURRENT));
        }
        else if (isSlammed() && (current == WristConstants.PERPENDICULAR_CURRENT)) {
            motor.setControl(controlRequest.withOutput(WristConstants.REDUCED_PERPENDICULAR_CURRENT));
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

}
