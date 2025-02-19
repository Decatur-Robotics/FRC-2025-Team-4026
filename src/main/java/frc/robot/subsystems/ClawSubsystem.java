package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
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

        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = ClawConstants.STATOR_CURRENT_LIMIT;

        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(motorConfigs);

        motor.optimizeBusUtilization();

        current = ClawConstants.CORAL_CURRENT;

        controlRequest = new TorqueCurrentFOC(current);
        motor.setControl(controlRequest);

        velocityFilter = LinearFilter.movingAverage(10);

        slamDebouncer = new Debouncer(ClawConstants.SLAM_DEBOUNCE_TIME);
    }

    @Override
    public void periodic() {
        if (motor.hasResetOccurred()) {
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
        return slamDebouncer.calculate(filteredVelocity < ClawConstants.MAX_SLAMMED_VELOCITY);
    }

}
