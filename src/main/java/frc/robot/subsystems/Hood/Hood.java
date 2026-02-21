package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Hood extends SubsystemBase{
    private TalonFX hoodMotor = new TalonFX(25);
    private CANcoder hoodEncoder = new CANcoder(26);
      protected TalonFXConfiguration config = new TalonFXConfiguration();
        private final  MotionMagicVoltage  positionOut = new MotionMagicVoltage(0);

    private static final double kSimDtSeconds = 0.02;
    private static final double kHoodGearRatio = 100.0;
    private static final double kHoodArmLengthMeters = 0.25;
    private static final double kHoodMassKg = 2.0;
    private static final double kHoodMinAngleRad = Units.degreesToRadians(-10.0);
    private static final double kHoodMaxAngleRad = Units.degreesToRadians(90.0);

    private double targetAngleDegrees = 0.0;
    public enum WantedState {
        IDLE,
        MOVING_TO_ANGLE,
    }
    public enum CurrentState {
        IDLE,
        MOVING_TO_ANGLE,        
    }

    public WantedState wantedState = WantedState.IDLE;
    public CurrentState currentState = CurrentState.IDLE;

    @AutoLog
    public static class HoodInputs {
        public double hoodAngleDegrees = 0.0;   
        public double targetAngleDegrees = 0.0;
        public double hoodAppliedVoltage = 0.0;
        public double hoodCurrent = 0.0;
        public String currentState = "IDLE";
        public String wantedState = "IDLE";
        public boolean atTargetAngle = false;
        public boolean hoodMotorConnected = false;
        public boolean hoodEncoderConnected = false;
    }

    private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();
    private TalonFXSimState hoodMotorSim;
    private CANcoderSimState hoodEncoderSim;
    private SingleJointedArmSim hoodArmSim;

    public Hood() {
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            config.Slot0.kG = 0;
            config.Slot0.kS = 0;
            config.Slot0.kP = 160;
            config.Slot0.kD = 0;
            config.MotionMagic.MotionMagicCruiseVelocity = 1;
            config.MotionMagic.MotionMagicAcceleration = 4;
            config.Feedback.withRemoteCANcoder(hoodEncoder);
            hoodMotor.getConfigurator().apply(config);

            if (RobotBase.isSimulation()) {
                hoodMotorSim = hoodMotor.getSimState();
                hoodEncoderSim = hoodEncoder.getSimState();
                hoodArmSim =
                        new SingleJointedArmSim(
                                DCMotor.getKrakenX60(1),
                                kHoodGearRatio,
                                SingleJointedArmSim.estimateMOI(kHoodArmLengthMeters, kHoodMassKg),
                                kHoodArmLengthMeters,
                                kHoodMinAngleRad,
                                kHoodMaxAngleRad,
                                true,
                                0.0);
            }
    }

    @Override
    public void periodic() {
        updateInputs();
        handleStates();
        applyStates();
        Logger.processInputs("Hood", inputs);
    }

    @Override
    public void simulationPeriodic() {
        if (hoodArmSim == null) {
            return;
        }

        double batteryVoltage = RobotController.getBatteryVoltage();
        hoodArmSim.setInputVoltage(hoodMotor.getMotorVoltage().getValueAsDouble());
        hoodArmSim.update(kSimDtSeconds);

        double armAngleRad = hoodArmSim.getAngleRads();
        double armVelocityRadPerSec = hoodArmSim.getVelocityRadPerSec();

        double armAngleRotations = Units.radiansToRotations(armAngleRad);
        double armVelocityRotationsPerSec = Units.radiansToRotations(armVelocityRadPerSec);

        hoodMotorSim.setSupplyVoltage(Volts.of(batteryVoltage));
        hoodMotorSim.setRawRotorPosition(Rotations.of(armAngleRotations * kHoodGearRatio));
        hoodMotorSim.setRotorVelocity(RotationsPerSecond.of(armVelocityRotationsPerSec * kHoodGearRatio));

        hoodEncoderSim.setSupplyVoltage(Volts.of(batteryVoltage));
        hoodEncoderSim.setRawPosition(Rotations.of(armAngleRotations));
        hoodEncoderSim.setVelocity(RotationsPerSecond.of(armVelocityRotationsPerSec));
    }

    private void updateInputs() {
        double hoodAngleRotations = hoodEncoder.getPosition().getValueAsDouble();
        inputs.hoodAngleDegrees = Units.rotationsToDegrees(hoodAngleRotations);
        inputs.targetAngleDegrees = targetAngleDegrees;
        inputs.hoodAppliedVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.hoodCurrent = hoodMotor.getStatorCurrent().getValueAsDouble();
        inputs.currentState = currentState.toString();
        inputs.wantedState = wantedState.toString();
        inputs.atTargetAngle = Math.abs(inputs.hoodAngleDegrees - targetAngleDegrees) < 1.0;
        inputs.hoodMotorConnected = hoodMotor.isConnected();
        inputs.hoodEncoderConnected = hoodEncoder.isConnected();
    }

    public boolean isAtTargetAngle() {
        double hoodAngleRotations = hoodEncoder.getPosition().getValueAsDouble();
        double hoodAngleDegrees = Units.rotationsToDegrees(hoodAngleRotations);
        return Math.abs(hoodAngleDegrees - targetAngleDegrees) < 1.0;
    }
      public boolean isAtTargetAngle(double targetangle) {
        double hoodAngleRotations = hoodEncoder.getPosition().getValueAsDouble();
        double hoodAngleDegrees = Units.rotationsToDegrees(hoodAngleRotations);
        return Math.abs(hoodAngleDegrees - targetangle) < 1.0;
    }

    public void handleStates() {
        switch (wantedState) {
            case IDLE:
                currentState = CurrentState.IDLE;
                break;
            case MOVING_TO_ANGLE:
                currentState = CurrentState.MOVING_TO_ANGLE;
                break;
        }
    }

    public void applyStates() {
        double targetRotations = Units.degreesToRotations(targetAngleDegrees);
        switch (currentState) {
            case IDLE:
                hoodMotor.setControl(positionOut.withPosition(0));
                break;
            case MOVING_TO_ANGLE:
                hoodMotor.setControl(positionOut.withPosition(targetRotations));
                break;
        }
    }
    public void moveToAngle(double angleDegrees) {
        targetAngleDegrees = angleDegrees;
        wantedState = WantedState.MOVING_TO_ANGLE;
    }

    public void setIdle() {
        targetAngleDegrees = 0.0;
        wantedState = WantedState.IDLE;
    }

    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }
    
}
