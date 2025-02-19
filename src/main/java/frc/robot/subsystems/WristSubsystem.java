package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = WristConstants.STATOR_CURRENT_LIMIT;

        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(motorConfigs);

        motor.optimizeBusUtilization();

        current = WristConstants.PARALLEL_CURRENT; 

        controlRequest = new TorqueCurrentFOC(current);
        motor.setControl(controlRequest.withOutput(current));

        velocityFilter = LinearFilter.movingAverage(10);

        slamDebouncer = new Debouncer(WristConstants.SLAM_DEBOUNCE_TIME);
    }

    @Override
    public void periodic() {
        if(motor.hasResetOccurred()) {
            motor.optimizeBusUtilization();
        }

        filteredVelocity = velocityFilter.calculate(motor.getVelocity().getValueAsDouble());
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
