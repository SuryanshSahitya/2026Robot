package frc.robot.commands;

import com.ctre.phoenix6.swerve.utility.LinearPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class LinearPathRequest2 extends Command {

    public Pose2d targetPose = Pose2d.kZero;

    // PID Controllers
    public PIDController xController = new PIDController(10, 0, 0);
    public PIDController yController = new PIDController(10, 0, 0);
    public PIDController thetaController = new PIDController(7, 0, 0);

    private final Drive drive;
    private final LinearPath path;

    // State
    private double elapsedTime = 0.0;
    private double lastTimestamp = 0.0;
    private LinearPath.State initialState = new LinearPath.State();
    private LinearPath.State setpoint = new LinearPath.State();

    public LinearPathRequest2(
            Drive drive,
            TrapezoidProfile.Constraints linearProfile,
            TrapezoidProfile.Constraints angularProfile) {
        this.drive = drive;
        this.path = new LinearPath(linearProfile, angularProfile);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    public void reset(Pose2d currentPose, ChassisSpeeds fieldSpeeds) {
        initialState = new LinearPath.State(currentPose, fieldSpeeds);
        elapsedTime = 0.0;
        setpoint = path.calculate(elapsedTime, initialState, targetPose);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();
        ChassisSpeeds fieldSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
        reset(currentPose, fieldSpeeds);

        xController.reset();
        yController.reset();
        thetaController.reset();
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double timestamp = Timer.getFPGATimestamp();
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;
        elapsedTime += dt;

        setpoint = path.calculate(elapsedTime, initialState, targetPose);

        Pose2d currentPose = drive.getPose();

        double xFeedback = xController.calculate(currentPose.getX(), setpoint.pose.getX());
        double yFeedback = yController.calculate(currentPose.getY(), setpoint.pose.getY());
        double thetaFeedback = thetaController.calculate(
                currentPose.getRotation().getRadians(),
                setpoint.pose.getRotation().getRadians());

        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(
                setpoint.speeds.vxMetersPerSecond + xFeedback,
                setpoint.speeds.vyMetersPerSecond + yFeedback,
                setpoint.speeds.omegaRadiansPerSecond + thetaFeedback);

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, drive.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return path.isFinished(elapsedTime);
    }

    public LinearPathRequest2 withTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        return this;
    }

    public LinearPathRequest2 withXController(PIDController controller) {
        this.xController = controller;
        return this;
    }

    public LinearPathRequest2 withYController(PIDController controller) {
        this.yController = controller;
        return this;
    }

    public LinearPathRequest2 withThetaController(PIDController controller) {
        this.thetaController = controller;
        return this;
    }
}
