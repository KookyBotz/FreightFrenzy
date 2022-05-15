package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.purepursuit.PurePursuit;
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
        waypoints.add(new Waypoint(20, 0, 12));
        waypoints.add(new Waypoint(20, 36, 12));
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

        if (current == waypoints.size() - 2 && robot.distanceTo(b) < radius) {
            this.robot.drive.tankDrive(0, 0);
        } else if (robot.distanceTo(b) < radius) {
            current++;
            a = waypoints.get(current);
            b = waypoints.get(current + 1);
        } else {
            Point target = PurePursuit.lineCircleIntersection(a, b, robot, radius);
            telemetry.addData("target: ", target.toString());
            double angle = Math.toDegrees(target.subtract(robot).atan()) - robot.angle;
            telemetry.addData("angle: ", angle);
            double angle_power = angle / PurePursuit.P_coefficients.angle;
            telemetry.addData("turn power: ", angle_power);
            telemetry.addData("turn power", angle_power);

            this.robot.drive.tankDrive(PurePursuit.default_power + angle_power, PurePursuit.default_power - angle_power);

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStroke("black")
                    .strokeLine(a.x, a.y, b.x, b.y)
                    .setStroke("blue")
                    .strokeCircle(robot.x, robot.y, radius)
                    .strokeCircle(robot.x, robot.y, 1)
                    .setStroke("red")
                    .strokeCircle(target.x, target.y, 1);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        telemetry.addData("Pose: ", odometry.getPoseMeters().toString());
        telemetry.update();
    }
}
