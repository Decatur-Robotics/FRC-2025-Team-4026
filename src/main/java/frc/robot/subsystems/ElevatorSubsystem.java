package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.ElevatorConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX motorFollower, motorMain;
    
    private double position;
    
    private MotionMagicVoltage positionControlRequest;

    private double voltage;

    private VoltageOut voltageControlRequest;

    private DigitalInput limitSwitch;

    private LinearFilter currentFilter;
    private double filteredCurrent;
    
    public ElevatorSubsystem() {
        motorFollower = new TalonFX(Ports.ELEVATOR_MOTOR_RIGHT);
        motorMain = new TalonFX(Ports.ELEVATOR_MOTOR_LEFT);

        motorFollower.setControl(new Follower(Ports.ELEVATOR_MOTOR_LEFT, true));

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;

        talonFXConfigs.Slot0.kP = ElevatorConstants.KP; 
        talonFXConfigs.Slot0.kI = ElevatorConstants.KI; 
        talonFXConfigs.Slot0.kD = ElevatorConstants.KD;
        talonFXConfigs.Slot0.kS = ElevatorConstants.KS; 
        talonFXConfigs.Slot0.kV = ElevatorConstants.KV;
        talonFXConfigs.Slot0.kA = ElevatorConstants.KA; 
        talonFXConfigs.Slot0.kG = ElevatorConstants.KG;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;

        motorMain.getConfigurator().apply(talonFXConfigs);
        motorFollower.getConfigurator().apply(talonFXConfigs);

        motorMain.optimizeBusUtilization();
        motorFollower.optimizeBusUtilization();
        motorMain.getRotorPosition().setUpdateFrequency(20);

        position = ElevatorConstants.INITIAL_POSITION;
        voltage = 0;

        positionControlRequest = new MotionMagicVoltage(position).withEnableFOC(true);
        motorMain.setControl(positionControlRequest);

        voltageControlRequest = new VoltageOut(voltage).withEnableFOC(true);

        currentFilter = LinearFilter.movingAverage(10);
    }

    @Override
    public void periodic() {
        if (motorMain.hasResetOccurred() || motorFollower.hasResetOccurred())
		{
			motorMain.optimizeBusUtilization();
			motorFollower.optimizeBusUtilization();
			motorMain.getRotorPosition().setUpdateFrequency(20);
		}

        filteredCurrent = currentFilter.calculate(getCurrent());
    }
 
    public void setPosition(double position) {
        this.position = position;
        motorMain.setControl(positionControlRequest.withPosition(position));
    }

    public double getPosition() {
        return motorMain.getRotorPosition().getValueAsDouble();
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public double getCurrent() {
        return motorMain.getStatorCurrent().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motorMain.setControl(voltageControlRequest.withOutput(voltage));
    }

    public Command zeroCommand() {
        Debouncer debouncer = new Debouncer(ElevatorConstants.STALL_DEBOUNCE_TIME, DebounceType.kRising);

        return Commands.sequence(
            Commands.runOnce(() -> setVoltage(ElevatorConstants.ZEROING_VOLTAGE)),
            Commands.waitUntil(() -> debouncer.calculate(filteredCurrent > ElevatorConstants.STALL_CURRENT))
        ).finallyDo(() -> {
            motorMain.setPosition(0);
            setPosition(ElevatorConstants.INITIAL_POSITION);
        });
    }

}
