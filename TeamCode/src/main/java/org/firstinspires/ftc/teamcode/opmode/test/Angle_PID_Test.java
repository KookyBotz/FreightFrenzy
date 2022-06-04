package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
@Config
public class Angle_PID_Test extends OpMode {

    private Robot robot;
    private DifferentialDriveOdometry odometry;
    private PIDController controller;

    public static double p, i, d, f, s, e;
    private double pp, pi, pd;

    public static double target;

    private double loop;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        odometry = new DifferentialDriveOdometry(new Rotation2d(0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller = new PIDController(p, i, d);

        d = 0.01;
        f = 0.15;
        i = 0;
        p = 0.002;
        s = 0.5;
        e = 35;
    }

    @Override
    public void loop() {
        if (p != pp || i != pi || d != pd) {
            controller.setPID(p, i, d);
        }
        pp = p;
        pi = i;
        pd = d;

        Rotation2d imu = new Rotation2d(-robot.imu.getAngularOrientation().firstAngle);
        double right_position = robot.right_encoder.getPosition() / 383.6 * 11.873736;
        double left_position = robot.left_encoder.getPosition() / 383.6 * 11.873736;

        odometry.update(
                imu, right_position, left_position
        );

        double voltageMultiplier = 12.0 / robot.batteryVoltageSensor.getVoltage();

        double error = AngleUnit.normalizeDegrees(target - Math.toDegrees(odometry.getPoseMeters().getHeading()));

        if (Math.abs(error) < 2) {
            error = 0;
        }

        double output = controller.calculate(0, error);
        output += Math.signum(error) * f;
        output *= voltageMultiplier;



        if (error < e) {
            robot.drive.tankDrive(output, -output);
        } else {
            robot.drive.tankDrive(Math.signum(error) * s, Math.signum(error) * -s);
        }

        double time = System.currentTimeMillis();
        telemetry.addData("loop ", time - loop);
        loop = time;

        telemetry.addData("error ", error);
        telemetry.addData("current", Math.toDegrees(odometry.getPoseMeters().getHeading()));
        telemetry.addData("target", target);
        telemetry.addData("power", output);
        telemetry.update();
    }
}
