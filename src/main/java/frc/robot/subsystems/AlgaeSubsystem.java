package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeSubsystemConstants;
import frc.robot.Constants.SimulationRobotConstants;

public class AlgaeSubsystem extends SubsystemBase {

    private final SparkMax armMotor =
        new SparkMax(AlgaeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);
    private final SparkClosedLoopController armController = armMotor.getClosedLoopController();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    private final SparkFlex intakeMotor =
        new SparkFlex(AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

    private boolean stowWhenIdle = true;
    private boolean wasReset = false;

    private final DCMotor armMotorModel = DCMotor.getNEO(1);
    private SparkMaxSim armMotorSim;
    private final SingleJointedArmSim m_intakeSim =
        new SingleJointedArmSim(
            armMotorModel,
            SimulationRobotConstants.kIntakeReduction,
            SingleJointedArmSim.estimateMOI(
                SimulationRobotConstants.kIntakeLength, SimulationRobotConstants.kIntakeMass),
            SimulationRobotConstants.kIntakeLength,
            SimulationRobotConstants.kIntakeMinAngleRads,
            SimulationRobotConstants.kIntakeMaxAngleRads,
            true,
            SimulationRobotConstants.kIntakeMinAngleRads,
            0.0,
            0.0);

    private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Ball Intake Root", 28, 3);
    private final MechanismLigament2d intakePivotMechanism =
        m_mech2dRoot.append(
            new MechanismLigament2d(
                "Intake Pivot",
                SimulationRobotConstants.kIntakeShortBarLength
                    * SimulationRobotConstants.kPixelsPerMeter,
                Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)));

    @SuppressWarnings("unused")
    private final MechanismLigament2d intakePivotSecondMechanism =
        intakePivotMechanism.append(
            new MechanismLigament2d(
                "Intake Pivot Second Bar",
                SimulationRobotConstants.kIntakeLongBarLength
                    * SimulationRobotConstants.kPixelsPerMeter,
                Units.radiansToDegrees(SimulationRobotConstants.kIntakeBarAngleRads)));

    public AlgaeSubsystem() {
        intakeMotor.configure(
            Configs.AlgaeSubsystem.intakeConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        armMotor.configure(
            Configs.AlgaeSubsystem.armConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        SmartDashboard.putData("Algae Subsystem", m_mech2d);

        // Safe initialization - force encoder and position to stow at startup
        initializeSafePosition();

        armMotorSim = new SparkMaxSim(armMotor, armMotorModel);
    }

    /** Ensures the arm starts in a safe stowed position to avoid collisions. */
    private void initializeSafePosition() {
        armEncoder.setPosition(AlgaeSubsystemConstants.ArmSetpoints.kStow);
        setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kStow);
    }

    private void zeroOnUserButton() {
        if (!wasReset && RobotController.getUserButton()) {
            wasReset = true;
            armEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasReset = false;
        }
    }

    public Command runIntakeCommand() {
        return this.run(() -> {
            stowWhenIdle = false;
            setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kForward);
            setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kDown);
        });
    }

    public Command reverseIntakeCommand() {
        return this.run(() -> {
            stowWhenIdle = true;
            setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kReverse);
            setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kHold);
        });
    }

    public Command stowCommand() {
        return this.runOnce(() -> stowWhenIdle = true);
    }

    public Command idleCommand() {
        return this.run(() -> {
            if (stowWhenIdle) {
                setIntakePower(0.0);
                setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kStow);
            } else {
                setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kHold);
                setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kHold);
            }
        });
    }

    private void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    private void setIntakePosition(double position) {
        armController.setReference(position, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        zeroOnUserButton();

        SmartDashboard.putNumber("Algae/Arm/Position", armEncoder.getPosition());
        SmartDashboard.putNumber("Algae/Intake/Applied Output", intakeMotor.getAppliedOutput());

        intakePivotMechanism.setAngle(
            Units.radiansToDegrees(SimulationRobotConstants.kIntakeMinAngleRads)
                + Units.rotationsToDegrees(
                    armEncoder.getPosition() / SimulationRobotConstants.kIntakeReduction));
    }

    public double getSimulationCurrentDraw() {
        return m_intakeSim.getCurrentDrawAmps();
    }

    @Override
    public void simulationPeriodic() {
        m_intakeSim.setInput(armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());
        m_intakeSim.update(0.020);

        armMotorSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(
                m_intakeSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
            RobotController.getBatteryVoltage(),
            0.02);
    }
}
