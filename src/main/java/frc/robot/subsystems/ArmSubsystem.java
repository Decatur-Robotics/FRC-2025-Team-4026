package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;

public class ArmSubsystem extends SubsystemBase {
	
	private TalonFX motor;
	
	private double position;

	private MotionMagicVoltage positionRequest;

	private VoltageOut voltageRequest;

	private CANcoder throughBoreEncoder;

	private double voltage;

	private double gravityFeedForward;

	private LinearFilter currentFilter;
    private double filteredCurrent;

	private Boolean zeroing;

	public ArmSubsystem() {
		motor = new TalonFX(Ports.ARM_MOTOR);

		motor.getConfigurator().apply(ArmConstants.MOTOR_CONFIG);

		motor.optimizeBusUtilization();
		motor.getPosition().setUpdateFrequency(20);
		motor.getVelocity().setUpdateFrequency(20);
        motor.getAcceleration().setUpdateFrequency(20);
		motor.getMotorVoltage().setUpdateFrequency(20);
		motor.getStatorCurrent().setUpdateFrequency(20);

		position = ArmConstants.STOWED_POSITION;

		positionRequest = new MotionMagicVoltage(position).withEnableFOC(true);

		setPosition(position);

		voltageRequest = new VoltageOut(0).withEnableFOC(true);

		// CANcoder uses custom configs for offset and direction
		throughBoreEncoder = new CANcoder(Ports.ARM_ENCODER);
		throughBoreEncoder.getConfigurator().apply(ArmConstants.ENCODER_CONFIG);

		configureShuffleboard();

		currentFilter = LinearFilter.movingAverage(10);

		voltage = 0;

		zeroing = false;
	}

	private void configureShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.SHUFFLEBOARD_SUPERSTRUCTURE_TAB);

		tab.addDouble("Target Arm Position", () -> position);
		tab.addDouble("Actual Arm Position", () -> getTalonPosition());
		tab.addDouble("Actual Arm Velocity", () -> motor.getVelocity().getValueAsDouble());
		tab.addDouble("Actual Arm Acceleration", () -> motor.getAcceleration().getValueAsDouble());
		tab.addDouble("Arm Through Bore Encoder Position", () -> getThroughBoreEncoderPosition());
		tab.addDouble("Arm Test Ratio", () -> (getTalonPosition()/getThroughBoreEncoderPosition()));
		tab.addDouble("Target Arm Voltage", () -> voltage);
		tab.addDouble("Actual Arm Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
		tab.addDouble("Gravity Feed Forward", () -> gravityFeedForward);
		tab.addDouble("Filtered Arm Current", () -> filteredCurrent);
	}
	
	@Override
	public void periodic() {
        if(motor.hasResetOccurred()) {
			motor.optimizeBusUtilization();
			motor.getPosition().setUpdateFrequency(20);
		}

		filteredCurrent = currentFilter.calculate(getCurrent());
	}

	public void setPosition(double position) {
		this.position = position;

		motor.setControl(positionRequest.withPosition(position));
	}

	public void setVoltage(double voltage) {
		this.voltage = voltage;
		motor.setControl(voltageRequest.withOutput(voltage));
	}

    public double getTalonPosition() {
        return motor.getPosition().getValueAsDouble();
    }

	public double getCurrent() {
		return motor.getStatorCurrent().getValueAsDouble();
	}

	/**
	 * @return through bore encoder position in rotations
	 */
	public double getThroughBoreEncoderPosition() {
		return throughBoreEncoder.getPosition().getValueAsDouble();
	}

	public Command zeroCommand() {
        Debouncer debouncer = new Debouncer(ArmConstants.STALL_DEBOUNCE_TIME, DebounceType.kFalling);

        return Commands.sequence(
			Commands.runOnce(() -> {zeroing = true;}),
            Commands.runOnce(() -> setVoltage(ArmConstants.ZEROING_VOLTAGE), this),
            Commands.waitUntil(() -> debouncer.calculate(Math.abs(filteredCurrent) > ArmConstants.STALL_CURRENT)),
            Commands.runOnce(() -> {
				motor.setPosition(1.5);
			}, this)
        ).finallyDo(() -> {
            setPosition(ArmConstants.STOWED_POSITION);
			zeroing = false;
        });
    }

	public Command setPositionCommand(double position) {
		return Commands.runOnce(() -> setPosition(position));
	}

	public Command setVoltageCommand(double voltage) {
		System.out.println("arm set voltage to " + voltage);
		return Commands.runEnd(() -> setVoltage(voltage), 
			() -> setVoltage(0),
			this);
	}

	public Command tuneVoltageCommand(Supplier<Double> voltage) {
        return Commands.run(() -> setVoltage(voltage.get()), this)
            .finallyDo(() -> setVoltage(0));
    }

	public Command setZeroPositionCommand() {
		return Commands.runOnce(() -> motor.setPosition(0), this);
	}

	// public Command resetTalonEncoderCommand() {
	// 	return Commands.runOnce(() -> resetTalonEncoder());
	// }

	/*
	 * SysId
	 */

	private final SysIdRoutine sysIdRoutine =
		new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.of(0.1).per(Second), // Quasistatic
				Volts.of(0.5), // Dynamic
				Seconds.of(20), // Timeout
				(state) -> SignalLogger.writeString("state", state.toString())
			),
			new SysIdRoutine.Mechanism(
				(volts) -> motor.setControl(voltageRequest.withOutput(volts.in(Volts))),
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
