package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.Ports;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFXS motorLeft, motorRight;

    private double velocity;
    private MotionMagicVelocityVoltage motorControlRequest;

    private LinearFilter currentFilter;
    private double filteredCurrent;
    
    public IntakeSubsystem() {
        motorLeft = new TalonFXS(Ports.INTAKE_MOTOR_LEFT);
        motorRight = new TalonFXS(Ports.INTAKE_MOTOR_RIGHT);

        motorRight.setControl(new Follower(motorLeft.getDeviceID(), true));

        TalonFXSConfiguration motorConfigs = new TalonFXSConfiguration();

        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.CURRENT_LIMIT;

        motorConfigs.Slot0.kP = IntakeConstants.KP;
        motorConfigs.Slot0.kI = IntakeConstants.KI;
        motorConfigs.Slot0.kD = IntakeConstants.KD;
        motorConfigs.Slot0.kS = IntakeConstants.KS;
        motorConfigs.Slot0.kV = IntakeConstants.KV;
        motorConfigs.Slot0.kA = IntakeConstants.KA;

        motorConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.ACCELERATION;
        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_VELOCITY;
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        motorLeft.getConfigurator().apply(motorConfigs);
        motorRight.getConfigurator().apply(motorConfigs);

        motorLeft.optimizeBusUtilization();
        motorRight.optimizeBusUtilization();
        motorLeft.getVelocity().setUpdateFrequency(20);
        
        velocity = IntakeConstants.REST_VELOCITY;
        
        motorControlRequest = new MotionMagicVelocityVoltage(velocity);

        currentFilter = LinearFilter.movingAverage(10);
    }

    @Override
    public void periodic() {
        filteredCurrent = currentFilter.calculate(getCurrent());
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
        motorLeft.setControl(motorControlRequest.withVelocity(velocity));
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
