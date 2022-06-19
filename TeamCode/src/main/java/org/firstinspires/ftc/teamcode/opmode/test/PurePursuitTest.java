package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Disabled

public class PurePursuitTest extends OpMode {
    private Robot robot;
    private DifferentialDriveOdometry odometry;

    private int current = 0;

    List<Waypoint> waypoints;

    private double loop = 0;

    private MotionProfile accel_profile;
    public static double max_a = 1;
    public static double max_v = 1;

    private ElapsedTime timeSinceStart;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        odometry = new DifferentialDriveOdometry(new Rotation2d(0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(0, 0, 10));
        waypoints.add(new Waypoint(18, 0, 10));
        waypoints.add(new Waypoint(36, 18, 10));
        waypoints.add(new Waypoint(36, 36, 10));

        accel_profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0), new MotionState(Integer.MAX_VALUE, 1), max_v, max_a);
    }

    @Override
    public void start() {
        timeSinceStart = new ElapsedTime();
    }

    @Override
    public void loop() {

        double time = System.currentTimeMillis();
        telemetry.addData("loop time ", time - loop);
        loop = time;

        Rotation2d imu = new Rotation2d(-robot.imu.getAngularOrientation().firstAngle);
        double right_position = robot.right_encoder.getPosition() / 383.6 * 11.873736;
        double left_position = robot.left_encoder.getPosition() / 383.6 * 11.873736;

        odometry.update(
                imu, right_position, left_position
        );

        Pose robot = new Pose(odometry.getPoseMeters());
        Waypoint a = waypoints.get(current);
        Waypoint b = waypoints.get(current + 1);
        double radius = b.radius;

        if (current == waypoints.size() - 2 && Math.abs(robot.distanceTo(b)) < PurePursuitUtil.admissible_error) {
            this.robot.drive.tankDrive(0, 0);
        } else if (current == waypoints.size() - 2 && Math.abs(robot.distanceTo(b)) < radius) {

            double angle = AngleUnit.normalizeRadians(b.subtract(robot).atan() - Math.toRadians(robot.angle));

            telemetry.addData("desired ", Math.toDegrees(AngleUnit.normalizeRadians(b.subtract(robot).atan())));
            telemetry.addData("actual ", robot.angle);
            if (b.reversed) {
                angle = AngleUnit.normalizeRadians(angle + Math.PI);
            }

            double angle_power = Math.max(-1, Math.min(1, angle / PurePursuitUtil.P_coefficients.angle));
            double forward_power = Math.max(-1, Math.min(1, robot.distanceTo(b) / PurePursuitUtil.P_coefficients.x));

            if (b.reversed) {
                forward_power *= -1;
            }

            double heading_scale = Math.abs(Math.cos(Math.min(Math.PI / 2, Math.max(-Math.PI / 2, angle))));

            double vel = accel_profile.get(timeSinceStart.seconds()).getV();
            telemetry.addData("accel ", vel);


            ArrayList<Double> powers = new ArrayList<>();
            powers.add((forward_power * heading_scale + angle_power) * vel);
            powers.add((forward_power * heading_scale - angle_power) * vel);


            this.robot.drive.tankDrive(powers.get(0), powers.get(1));

            Point heading_line = new Point(robot.x, robot.y).rotated(Math.toRadians(-robot.angle)).add(new Point(10, 0)).rotated(Math.toRadians(robot.angle));

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStroke("black")
                    .strokeLine(a.x, a.y, b.x, b.y)
                    .setStroke("blue")
                    .strokeCircle(robot.x, robot.y, radius)
                    .strokeCircle(robot.x, robot.y, 1)
                    .strokeLine(robot.x, robot.y, heading_line.x, heading_line.y)
                    .setStroke("red")
                    .strokeCircle(b.x, b.y, 1);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        } else if (Math.abs(robot.distanceTo(b)) < radius) {
            current++;
            a = waypoints.get(current);
            b = waypoints.get(current + 1);
        } else {
            Point target = PurePursuitUtil.lineCircleIntersection(a, b, robot, radius);

            double angle = AngleUnit.normalizeRadians(target.subtract(robot).atan() - Math.toRadians(robot.angle));

            telemetry.addData("desired ", Math.toDegrees(AngleUnit.normalizeRadians(target.subtract(robot).atan())));
            telemetry.addData("actual ", robot.angle);
            if (b.reversed) {
                angle = AngleUnit.normalizeRadians(angle + Math.PI);
            }

            double angle_power = Math.max(-1, Math.min(1, angle / PurePursuitUtil.P_coefficients.angle));
            double forward_power = Math.max(-1, Math.min(1, robot.distanceTo(target) / PurePursuitUtil.P_coefficients.x));

            if (b.reversed) {
                forward_power *= -1;
            }

            double heading_scale = Math.abs(Math.cos(Math.min(Math.PI / 2, Math.max(-Math.PI / 2, angle))));

            double vel = accel_profile.get(timeSinceStart.seconds()).getV();
            telemetry.addData("accel ", vel);


            ArrayList<Double> powers = new ArrayList<>();
            powers.add((forward_power * heading_scale + angle_power) * vel);
            powers.add((forward_power * heading_scale - angle_power) * vel);


            this.robot.drive.tankDrive(powers.get(0), powers.get(1));

            Point heading_line = new Point(robot.x, robot.y).rotated(Math.toRadians(-robot.angle)).add(new Point(10, 0)).rotated(Math.toRadians(robot.angle));

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStroke("black")
                    .strokeLine(a.x, a.y, b.x, b.y)
                    .setStroke("blue")
                    .strokeCircle(robot.x, robot.y, radius)
                    .strokeCircle(robot.x, robot.y, 1)
                    .strokeLine(robot.x, robot.y, heading_line.x, heading_line.y)
                    .setStroke("red")
                    .strokeCircle(target.x, target.y, 1);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        telemetry.addData("Pose: ", odometry.getPoseMeters().toString());
        telemetry.update();
    }
}