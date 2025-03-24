package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Setpoints;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.commands.*;

public class CoralSubsystem extends SubsystemBase {
    private final ArmSubsystem armSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final SparkMax intakeMotor;

    public CoralSubsystem(int armMotorID, int elevatorMotorID, int intakeMotorID, int elevatorLimitSwitchChannel) {
        armSubsystem = new ArmSubsystem(armMotorID);
        elevatorSubsystem = new ElevatorSubsystem(elevatorMotorID, elevatorLimitSwitchChannel);
        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
        intakeMotor.configure(Configs.CoralSubsystem.intakeConfig, SparkMax.ResetMode.kNoResetSafeParameters, SparkMax.PersistMode.kNoPersistParameters);
    }

    public void setArmPosition(double position) {
        armSubsystem.setPosition(position);
    }

    public void setElevatorPosition(double position) {
        elevatorSubsystem.setPosition(position);
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    private void setIntakePower(double power) {
        intakeMotor.set(power);
    }
    
    public void runIntake() {
        setIntakePower(CoralSubsystemConstants.IntakeSetpoints.kForward);
    }

    public void reverseIntake() {
        setIntakePower(CoralSubsystemConstants.IntakeSetpoints.kReverse);
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public Command runIntakeCommand() {
        return new RunCommand(() -> runIntake(), this);
    }

    public Command reverseIntakeCommand() {
        return new RunCommand(() -> reverseIntake(), this);
    }

    public void moveToSetpoint(Setpoints.Setpoint setpoint) {
        switch (setpoint) {
            case kFeederStation:
                setArmPosition(Setpoints.kFeederStationArm);
                setElevatorPosition(Setpoints.kFeederStationElevator);
                break;
            case kLevel1:
                setArmPosition(Setpoints.kLevel1Arm);
                setElevatorPosition(Setpoints.kLevel1Elevator);
                break;
            case kLevel2:
                setArmPosition(Setpoints.kLevel2Arm);
                setElevatorPosition(Setpoints.kLevel2Elevator);
                break;
            case kLevel3:
                setArmPosition(Setpoints.kLevel3Arm);
                setElevatorPosition(Setpoints.kLevel3Elevator);
                break;
            case kLevel4:
                setArmPosition(Setpoints.kLevel4Arm);
                setElevatorPosition(Setpoints.kLevel4Elevator);
                break;
        }
    }
}




/* 
public class CoralSubsystem extends SubsystemBase {

    private final SparkMax armMotor = new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private boolean isManualControl = false;

    private final SparkMax elevatorMotor = new SparkMax(CoralSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    private final SparkMax intakeMotor = new SparkMax(CoralSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

    private final DigitalInput elevatorBottomSwitch = new DigitalInput(9);

    private boolean wasResetByButton = false;
    private boolean wasResetByLimit = false;
    private double armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kFeederStation;
    private double elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation;

    public CoralSubsystem() {
        armMotor.restoreFactoryDefaults();
        elevatorMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        armMotor.setInverted(Configs.CoralSubsystem.armConfig.isInverted());
        elevatorMotor.setInverted(Configs.CoralSubsystem.elevatorConfig.isInverted());
        intakeMotor.setInverted(Configs.CoralSubsystem.intakeConfig.isInverted());

        armEncoder.setPosition(0);
        elevatorEncoder.setPosition(0);
    }

    public void resetArmEncoder() {
        armEncoder.setPosition(0);
    }

    public void enableManualControl() {
        isManualControl = true;
    }

    public void moveArmManually(double joystickInput) {
        double speed = joystickInput * 0.3; // Scale joystick input to control speed
        if (Math.abs(speed) > 0.1) { // Deadband to prevent small accidental movement
            armMotor.set(speed);
        } else {
            armMotor.set(0); // Stop motor if joystick input is idle
        }
    }

    public void setArmPosition(double position) {
        isManualControl = false;
        // Implement PID control or motion profiling to move arm to the desired position
        // Example: armController.setReference(position, ControlType.kPosition);
    }

    public void setElevatorPosition(double position) {
        // Implement PID control or motion profiling to move elevator to the desired position
        // Example: elevatorController.setReference(position, ControlType.kPosition);
    }

    private void zeroElevatorOnLimitSwitch() {
        if (!wasResetByLimit && !elevatorBottomSwitch.get()) {
            elevatorEncoder.setPosition(0);
            wasResetByLimit = true;
        } else if (elevatorBottomSwitch.get()) {
            wasResetByLimit = false;
        }
    }

    private void zeroOnUserButton() {
        if (!wasResetByButton && RobotController.getUserButton()) {
            wasResetByButton = true;
            armEncoder.setPosition(0);
            elevatorEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
    }

    private void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public void setSetpoint(Setpoint setpoint) {
        switch (setpoint) {
            case kFeederStation:
                armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kFeederStation;
                elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kFeederStation;
                break;
            case kLevel1:
                armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kLevel1;
                elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kLevel1;
                break;
            case kLevel2:
                armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kLevel2;
                elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kLevel2;
                break;
            case kLevel3:
                armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kLevel3;
                elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kLevel3;
                break;
            case kLevel4:
                armCurrentTarget = CoralSubsystemConstants.ArmSetpoints.kLevel4;
                elevatorCurrentTarget = CoralSubsystemConstants.ElevatorSetpoints.kLevel4;
                break;
        }
        setArmPosition(armCurrentTarget);
        setElevatorPosition(elevatorCurrentTarget);
    }

    public void runIntake() {
        setIntakePower(CoralSubsystemConstants.IntakeSetpoints.kForward);
    }

    public void reverseIntake() {
        setIntakePower(CoralSubsystemConstants.IntakeSetpoints.kReverse);
    }

    @Override
    public void periodic() {
        zeroElevatorOnLimitSwitch();
        zeroOnUserButton();

        SmartDashboard.putNumber("Coral/Arm/Position", armEncoder.getPosition());
        SmartDashboard.putNumber("Coral/Elevator/Position", elevatorEncoder.getPosition());
        SmartDashboard.putBoolean("Coral/Elevator/Bottom Switch", !elevatorBottomSwitch.get());
    }
}
*/