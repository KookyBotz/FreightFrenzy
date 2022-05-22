package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.PurePursuitUtil;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class PurePursuitTest extends OpMode {
    private Robot robot;
    private DifferentialDriveOdometry odometry;

    private int current = 0;

    List<Waypoint> waypoints;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        odometry = new DifferentialDriveOdometry(new Rotation2d(0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(0, 0, 0));
        waypoints.add(new Waypoint(20, 0, 8.5));
        waypoints.add(new Waypoint(20, 36, 8.5));


    }

    @Override
    public void loop() {

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

        if (current == waypoints.size() - 2 && Math.abs(robot.distanceTo(b)) < radius) {
            this.robot.drive.tankDrive(0, 0);
        } else if (Math.abs(robot.distanceTo(b)) < radius) {
            current++;
            a = waypoints.get(current);
            b = waypoints.get(current + 1);
        } else {
            Point target = PurePursuitUtil.lineCircleIntersection(a, b, robot, radius);

            double x = PurePursuitUtil.getX(robot, target);
            double curvature = PurePursuitUtil.getCurvature(x, radius);
            double sign = PurePursuitUtil.getSign(robot, target);

            List<Double> powers = PurePursuitUtil.calculateWheelSpeeds(robot, target, curvature, sign, 0.2, 8.3);


            telemetry.addData("tan", Math.tan(Math.toRadians(robot.angle)));
            telemetry.addData("x ", x);
            telemetry.addData("c ", curvature);
            telemetry.addData("signed ", sign);
            telemetry.addData("p ", powers.toString());
            this.robot.drive.tankDrive(powers.get(1), powers.get(0));

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
