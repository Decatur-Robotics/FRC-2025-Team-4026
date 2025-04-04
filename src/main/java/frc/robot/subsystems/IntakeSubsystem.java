package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.Ports;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX motorLeft, motorRight;

    private double velocity;
    private VelocityVoltage velocityRequest;
    private VoltageOut voltageRequest;

    private LinearFilter currentFilterLeft;
    private double filteredCurrentLeft;
    private LinearFilter currentFilterRight;
    private double filteredCurrentRight;

    private double voltage;
    
    public IntakeSubsystem() {
        motorLeft = new TalonFX(Ports.INTAKE_MOTOR_LEFT);
        motorRight = new TalonFX(Ports.INTAKE_MOTOR_RIGHT);
        
        motorLeft.getConfigurator().apply(IntakeConstants.MOTOR_LEFT_CONFIG);

        motorRight.getConfigurator().apply(IntakeConstants.MOTOR_RIGHT_CONFIG);

        motorLeft.optimizeBusUtilization();
        motorRight.optimizeBusUtilization();
        motorLeft.getVelocity().setUpdateFrequency(20);
        motorRight.getVelocity().setUpdateFrequency(20);
        motorLeft.getStatorCurrent().setUpdateFrequency(20);
        motorRight.getStatorCurrent().setUpdateFrequency(20);
        motorLeft.getMotorVoltage().setUpdateFrequency(20);
        motorRight.getMotorVoltage().setUpdateFrequency(20);
        motorLeft.getStatorCurrent().setUpdateFrequency(20);
        motorRight.getStatorCurrent().setUpdateFrequency(20);
        
        velocity = IntakeConstants.CORAL_REST_VELOCITY;
        
        velocityRequest = new VelocityVoltage(velocity);

        motorLeft.setControl(velocityRequest.withVelocity(velocity));
        motorRight.setControl(velocityRequest.withVelocity(velocity));

        voltage = 0;

        voltageRequest = new VoltageOut(voltage);

        currentFilterLeft = LinearFilter.movingAverage(10);
        currentFilterRight = LinearFilter.movingAverage(10);

        configureShuffleboard();
    }

    private void configureShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab(Constants.SHUFFLEBOARD_SUPERSTRUCTURE_TAB);

        tab.addDouble("Target Intake Velocity", () -> velocity);
        tab.addDouble("Actual Intake Velocity", () -> getVelocity());
        tab.addDouble("Filtered Intake Current", () -> filteredCurrentLeft);
        tab.addDouble("Target Intake Voltage", () -> voltage);
        tab.addDouble("Actual Intake Voltage", () -> motorLeft.getMotorVoltage().getValueAsDouble());
        tab.addDouble("Intake Current", () -> motorLeft.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void periodic() {
        filteredCurrentLeft = currentFilterLeft.calculate(getCurrentLeft());
        filteredCurrentRight = currentFilterRight.calculate(getCurrentRight());
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
        motorLeft.setControl(velocityRequest.withVelocity(velocity));
        motorRight.setControl(velocityRequest.withVelocity(velocity));
    }

    public double getVelocity() {
        return motorLeft.getVelocity().getValueAsDouble();
    }

    public double getCurrentLeft() {
        return motorLeft.getStatorCurrent().getValueAsDouble();
    }
    
    public double getCurrentRight() {
        return motorRight.getStatorCurrent().getValueAsDouble();
    }

    public double getFilteredCurrentLeft() {
        return filteredCurrentLeft;
    }

    public double getFilteredCurrentRight() {
        return filteredCurrentRight;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        motorLeft.setControl(voltageRequest.withOutput(voltage));
        motorRight.setControl(voltageRequest.withOutput(voltage));
    }

    public Command setVelocityCommand(double velocity) {
        return Commands.run(() -> setVelocity(velocity)) 
            .finallyDo(() -> setVelocity(IntakeConstants.CORAL_REST_VELOCITY));
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.run(() -> setVoltage(voltage), this)
            .finallyDo(() -> setVoltage(0));
    }

}
