package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.Ports;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFXS motorLeft, motorRight;

    private double velocity;
    private VelocityVoltage controlRequest;

    private LinearFilter currentFilter;
    private double filteredCurrent;
    
    public IntakeSubsystem() {
        motorLeft = new TalonFXS(Ports.INTAKE_MOTOR_LEFT);
        motorRight = new TalonFXS(Ports.INTAKE_MOTOR_RIGHT);

        TalonFXSConfiguration motorConfigs = new TalonFXSConfiguration();

        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.CURRENT_LIMIT;

        motorConfigs.Slot0.kP = IntakeConstants.KP;
        motorConfigs.Slot0.kI = IntakeConstants.KI;
        motorConfigs.Slot0.kD = IntakeConstants.KD;
        motorConfigs.Slot0.kS = IntakeConstants.KS;
        motorConfigs.Slot0.kV = IntakeConstants.KV;
        motorConfigs.Slot0.kA = IntakeConstants.KA;

        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        motorLeft.getConfigurator().apply(motorConfigs);

        motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorRight.getConfigurator().apply(motorConfigs);

        motorLeft.optimizeBusUtilization();
        motorRight.optimizeBusUtilization();
        motorLeft.getVelocity().setUpdateFrequency(20);
        motorRight.getVelocity().setUpdateFrequency(20);
        
        velocity = IntakeConstants.REST_VELOCITY;
        
        controlRequest = new VelocityVoltage(velocity);

        motorLeft.setControl(controlRequest.withVelocity(velocity));
        motorRight.setControl(controlRequest.withVelocity(velocity));

        currentFilter = LinearFilter.movingAverage(10);
    }

    @Override
    public void periodic() {
        filteredCurrent = currentFilter.calculate(getCurrent());
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
        motorLeft.setControl(controlRequest.withVelocity(velocity));
        motorRight.setControl(controlRequest.withVelocity(velocity));
    }

    public double getVelocity() {
        return motorLeft.getVelocity().getValueAsDouble();
    }

    public double getCurrent() {
        return motorLeft.getStatorCurrent().getValueAsDouble();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }

}
