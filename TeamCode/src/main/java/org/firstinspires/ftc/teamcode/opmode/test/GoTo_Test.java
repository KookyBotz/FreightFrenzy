package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
@Config

public class GoTo_Test extends OpMode {

    private Robot robot;
    private DifferentialDriveOdometry odometry;
    private PIDController angleController, distanceController;

    public static double p, i, d;
    public static double p2, i2, d2;

    public static double targetX, targetY, targetT;

    private double loop;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, false);
        odometry = new DifferentialDriveOdometry(new Rotation2d(0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        p = 0.005;
        d = 0.001;
        p2 = 0.061;

        angleController = new PIDController(p, i, d);
        distanceController = new PIDController(p2, i2, d2);
    }

    @Override
    public void loop() {
        angleController.setPID(p, i, d);
        distanceController.setPID(p2, i2, d2);


        Rotation2d imu = new Rotation2d(-robot.imu.getAngularOrientation().firstAngle);
        double right_position = robot.right_encoder.getPosition() / 383.6 * 11.873736;
        double left_position = robot.left_encoder.getPosition() / 383.6 * 11.873736;

        odometry.update(
                imu, right_position, left_position
        );

        Pose2d robot = odometry.getPoseMeters();

        double voltageMultiplier = 12.0 / this.robot.batteryVoltageSensor.getVoltage();

        double xError = targetX - robot.getX();
        double yError = targetY - robot.getY();
        double theta = Math.toDegrees(Math.atan2(yError, xError));
        // 0 is the reference because we want the distance to go to 0
        double distance = Math.hypot(xError, yError);
        double tError = AngleUnit.normalizeDegrees(theta - Math.toDegrees(robot.getHeading()));

        double f, t;

        if (distance > 2) {
            if (Math.abs(tError) > 90) {
                tError = AngleUnit.normalizeDegrees(tError + 180);
                distance *= -1;
            }

            f = distanceController.calculate(0, distance);
            t = angleController.calculate(0, tError);
        } else {
            f = 0;
            t = 0;
            tError = AngleUnit.normalizeDegrees(targetT - Math.toDegrees(robot.getHeading()));
            if(Math.abs(tError) > 1){
                t = angleController.calculate(0, tError);
                t += Math.signum(t) * 0.12;
            }
        }

        f *= Math.cos(Math.max(Math.min(Math.toRadians(tError), Math.PI / 2), -Math.PI / 2));

        this.robot.drive.tankDrive((f + t) * voltageMultiplier, (f - t) * voltageMultiplier);

        telemetry.addData("xError ", xError);
        telemetry.addData("yError ", yError);
        telemetry.addData("angle To target ", theta);
        telemetry.addData("angle error ", tError);
        telemetry.addData("f ", f);
        telemetry.addData("t ", t);
        telemetry.update();
    }
}
