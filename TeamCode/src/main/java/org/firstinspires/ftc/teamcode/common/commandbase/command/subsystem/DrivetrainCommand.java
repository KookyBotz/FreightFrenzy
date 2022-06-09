package org.firstinspires.ftc.teamcode.common.commandbase.command.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

public class DrivetrainCommand extends CommandBase {
    private Robot robot;
    private DifferentialDriveOdometry odometry;
    private PIDController angleController, distanceController;
    private Pose target;
    private Pose robotPose;

    private Telemetry telemetry;

    private double stablems = 0;
    private boolean atTarget = false;
    private ElapsedTime stableTimer;

    private boolean jitterhack = false;

    public DrivetrainCommand(Pose target, Robot robot, DifferentialDriveOdometry odometry, Telemetry telemetry) {
        this.robot = robot;
        this.target = target;
        this.odometry = odometry;
        this.telemetry = telemetry;

        angleController = new PIDController(0.0065, 0, 0);
        distanceController = new PIDController(0.05, 0, 0);
    }

    public DrivetrainCommand(Pose target, Robot robot, DifferentialDriveOdometry odometry, Telemetry telemetry, double stablems) {
        this(target, robot, odometry, telemetry);
        this.stablems = stablems;
    }

    @Override
    public void execute() {
        robotPose = new Pose(odometry.getPoseMeters());

        double voltageMultiplier = 12.0 / this.robot.batteryVoltageSensor.getVoltage();

        double xError = target.x - robotPose.x;
        double yError = target.y - robotPose.y;
        double theta = Math.toDegrees(Math.atan2(yError, xError));
        double distance = Math.hypot(xError, yError);
        double tError = AngleUnit.normalizeDegrees(theta - robotPose.angle);

        double f, t;

        if (distance > 2 && !jitterhack) {
            if (Math.abs(tError) > 90) {
                tError = AngleUnit.normalizeDegrees(tError + 180);
                distance *= -1;
            }

            f = distanceController.calculate(0, distance);
            t = angleController.calculate(0, tError);
        } else {
            jitterhack = true;
            f = 0;
            t = 0;
            tError = AngleUnit.normalizeDegrees(target.angle - robotPose.angle);
            if (Math.abs(tError) > 1) {
                t = angleController.calculate(0, tError);
                t += Math.signum(t) * 0.12;
            }
        }

        f *= Math.cos(Math.max(Math.min(Math.toRadians(tError * 3), Math.PI / 2), -Math.PI / 2));

        f = Range.clip(f, -0.5, 0.5);
        t = Range.clip(t, -0.5, 0.5);

        this.robot.drive.tankDrive((f + t) * voltageMultiplier, (f - t) * voltageMultiplier);

        telemetry.addData("xError ", xError);
        telemetry.addData("yError ", yError);
        telemetry.addData("angle To target ", theta);
        telemetry.addData("angle error ", tError);
        telemetry.addData("f ", f);
        telemetry.addData("t ", t);
    }

    @Override
    public boolean isFinished() {
        if (jitterhack && Math.abs(AngleUnit.normalizeDegrees(target.angle - robotPose.angle)) < 2 && stableTimer == null) {
            atTarget = true;
            stableTimer = new ElapsedTime();
        }

        if (atTarget && stableTimer.milliseconds() >= stablems) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.robot.drive.tankDrive(0, 0);
    }
}
