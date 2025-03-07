package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

	private DutyCycleEncoder throughBoreEncoder;

	private double voltage;

	private double gravityFeedForward;

	public ArmSubsystem() {
		motor = new TalonFX(Ports.ARM_MOTOR);

		motor.getConfigurator().apply(ArmConstants.MOTOR_CONFIG);

		motor.optimizeBusUtilization();
		motor.getPosition().setUpdateFrequency(20);
		motor.getVelocity().setUpdateFrequency(20);
        motor.getAcceleration().setUpdateFrequency(20);
		motor.getMotorVoltage().setUpdateFrequency(20);

		position = ArmConstants.STOWED_POSITION;

		positionRequest = new MotionMagicVoltage(position).withEnableFOC(true);

		setPosition(position);

		voltageRequest = new VoltageOut(0).withEnableFOC(true);

		// k4X is quadrature encoding
		throughBoreEncoder = new DutyCycleEncoder(Ports.ARM_ENCODER);

		// resetTalonEncoder();

		configureShuffleboard();

		voltage = 0;
	}

	private void configureShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.SHUFFLEBOARD_SUPERSTRUCTURE_TAB);

		tab.addDouble("Target Arm Position", () -> position);
		tab.addDouble("Actual Arm Position", () -> getTalonPosition());
		tab.addDouble("Actual Arm Velocity", () -> motor.getVelocity().getValueAsDouble());
		tab.addDouble("Actual Arm Acceleration", () -> motor.getAcceleration().getValueAsDouble());
		tab.addDouble("Arm Through Bore Encoder Position", () -> getThroughBoreEncoderPosition());
		tab.addDouble("Arm Test Ratio", () -> (getTalonPosition()/getThroughBoreEncoderPosition()));
		tab.addDouble("Arm Position Calc", () -> (getThroughBoreEncoderPosition() / ArmConstants.TALON_ENCODER_TO_ROTATIONS_RATIO));
		tab.addDouble("Target Arm Voltage", () -> voltage);
		tab.addDouble("Actual Arm Voltage", () -> motor.getMotorVoltage().getValueAsDouble());
		tab.addDouble("Gravity Feed Forward", () -> gravityFeedForward);
	}
	
	@Override
	public void periodic() {
        if(motor.hasResetOccurred()) {
			motor.optimizeBusUtilization();
			motor.getPosition().setUpdateFrequency(20);
		}

		Rotation2d angle = Rotation2d.fromRotations(getThroughBoreEncoderPosition());

		gravityFeedForward = Math.cos(angle.getRadians()) * ArmConstants.KG;

		// motor.setControl(positionRequest.withFeedForward(gravityFeedForward));
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

	/**
	 * @return through bore encoder position in rotations
	 */
	public double getThroughBoreEncoderPosition() {
		return (throughBoreEncoder.get()) - ArmConstants.THROUGH_BORE_ENCODER_ZERO_OFFSET;
	}

	// public void resetTalonEncoder() {
    //     double rotations = getThroughBoreEncoderPosition() / ArmConstants.TALON_ENCODER_TO_ROTATIONS_RATIO;
	// 	motor.setPosition(rotations);
    // }

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

	public Command zeroArmCommand() {
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
				Volts.of(1).per(Second), // Quasistatic
				Volts.of(4), // Dynamic
				Seconds.of(10), // Timeout
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
