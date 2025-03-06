package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Ports;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX motorFollower, motorMain;
    
    private double position;
    
    private MotionMagicVoltage positionRequest;
    private VelocityVoltage velocityRequest;

    private double voltage;

    private VoltageOut voltageRequest;

    private DigitalInput limitSwitch;

    private LinearFilter currentFilter;
    private double filteredCurrent;
    
    public ElevatorSubsystem() {
        motorFollower = new TalonFX(Ports.ELEVATOR_MOTOR_RIGHT);
        motorMain = new TalonFX(Ports.ELEVATOR_MOTOR_LEFT);

        motorMain.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);
        motorFollower.getConfigurator().apply(ElevatorConstants.MOTOR_CONFIG);

        motorMain.optimizeBusUtilization();
        motorFollower.optimizeBusUtilization();
        motorMain.getPosition().setUpdateFrequency(20);
        motorMain.getVelocity().setUpdateFrequency(20);
        motorMain.getAcceleration().setUpdateFrequency(20);
        motorMain.getMotorVoltage().setUpdateFrequency(20);
        motorMain.getStatorCurrent().setUpdateFrequency(20);

        position = ElevatorConstants.STOWED_POSITION;
        voltage = 0;

        motorFollower.setControl(new Follower(motorMain.getDeviceID(), true));

        positionRequest = new MotionMagicVoltage(position).withEnableFOC(true);
        motorMain.setControl(positionRequest);

        voltageRequest = new VoltageOut(voltage).withEnableFOC(true);
        velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

        currentFilter = LinearFilter.movingAverage(10);

        configureShuffleboard();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.SHUFFLEBOARD_SUPERSTRUCTURE_TAB);

        tab.addDouble("Filtered Elevator Current", () -> filteredCurrent);
        tab.addDouble("Target Elevator Position", () -> position);
        tab.addDouble("Actual Elevator Position", () -> getPosition());
		tab.addDouble("Actual Elevator Velocity", () -> motorMain.getVelocity().getValueAsDouble());
		tab.addDouble("Actual Elevator Acceleration", () -> motorMain.getAcceleration().getValueAsDouble());
        tab.addDouble("Target Elevator Voltage", () -> voltage);
        tab.addDouble("Actual Elevator Voltage", () -> motorMain.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (motorMain.hasResetOccurred() || motorFollower.hasResetOccurred())
		{
			motorMain.optimizeBusUtilization();
			motorFollower.optimizeBusUtilization();
			motorMain.getPosition().setUpdateFrequency(20);
		}

        filteredCurrent = currentFilter.calculate(getCurrent());
    }
 
    public void setPosition(double position) {
        this.position = position;
        motorMain.setControl(positionRequest.withPosition(position));
    }

    public double getPosition() {
        return motorMain.getPosition().getValueAsDouble();
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public double getCurrent() {
        return motorMain.getStatorCurrent().getValueAsDouble();
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motorMain.setControl(voltageRequest.withOutput(voltage));
    }

    public Command zeroCommand() {
        Debouncer debouncer = new Debouncer(ElevatorConstants.STALL_DEBOUNCE_TIME, DebounceType.kRising);

        return Commands.sequence(
            Commands.runOnce(() -> setVoltage(ElevatorConstants.ZEROING_VOLTAGE)),
            Commands.waitUntil(() -> debouncer.calculate(filteredCurrent > Math.abs(ElevatorConstants.STALL_CURRENT))),
            Commands.runOnce(() -> motorMain.setPosition(0))
        ).finallyDo(() -> {
            setPosition(ElevatorConstants.STOWED_POSITION);
        });
    }

    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.run(() -> setVoltage(voltage), this)
            .finallyDo(() -> setVoltage(0));
    }

    public Command tuneVoltageCommand(Supplier<Double> voltage) {
        return Commands.run(() -> setVoltage(voltage.get()), this)
            .finallyDo(() -> setVoltage(0));
    }

    // public Command setVelocityCommand(double velocity) {
    //     return Commands.run(() -> motorMain.setControl(velocityRequest.withVelocity(velocity)));
    // }

    /*
	 * SysId
	 */

	private final SysIdRoutine sysIdRoutine =
		new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.of(0.4).per(Second), // Quasistatic
				Volts.of(1.5), // Dynamic
				Seconds.of(10), // Timeout
				(state) -> SignalLogger.writeString("state", state.toString())
			),
			new SysIdRoutine.Mechanism(
				(volts) -> motorMain.setControl(voltageRequest.withOutput(volts.in(Volts))),
				null,
				this
			)
		);

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}
	 
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

}
