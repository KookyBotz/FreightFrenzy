package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

public class PurePursuitPath {
    private final LinkedList<Waypoint> waypoints;
    private int current = 0;
    private boolean finished = false;

    private MotionProfile accel_profile;
    public static double max_a = 1;
    public static double max_v = 1;

    private ElapsedTime timeSinceStart;


    public PurePursuitPath(Waypoint... w) {
        waypoints = new LinkedList<>(Arrays.asList(w));

        if (waypoints.size() < 2) throw new IllegalArgumentException("Bro....");

        accel_profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0), new MotionState(Integer.MAX_VALUE, 1), max_v, max_a);
    }

    public void init() {
        timeSinceStart = new ElapsedTime();
    }

    public List<Double> update(Pose robot) {
        if (finished) return new ArrayList<>(Collections.nCopies(2, 0.0));

        Waypoint a = waypoints.get(current);
        Waypoint b = waypoints.get(current + 1);
        double radius = b.radius;

        if (current == waypoints.size() - 2 && Math.abs(robot.distanceTo(b)) < PurePursuitUtil.admissible_error) {
            finished = true;
            return new ArrayList<>(Collections.nCopies(2, 0.0));
        } else if (current == waypoints.size() - 2 && Math.abs(robot.distanceTo(b)) < radius) {

            double angle = AngleUnit.normalizeRadians(b.subtract(robot).atan() - Math.toRadians(robot.angle));

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

            ArrayList<Double> powers = new ArrayList<>();
            powers.add((forward_power * heading_scale + angle_power) * vel);
            powers.add((forward_power * heading_scale - angle_power) * vel);


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

            return powers;
        } else if (Math.abs(robot.distanceTo(b)) < radius) {
            current++;
            a = waypoints.get(current);
            b = waypoints.get(current + 1);
        }

        Point target = PurePursuitUtil.lineCircleIntersection(a, b, robot, radius);

        double angle = AngleUnit.normalizeRadians(target.subtract(robot).atan() - Math.toRadians(robot.angle));

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


        ArrayList<Double> powers = new ArrayList<>();
        powers.add((forward_power * heading_scale + angle_power) * vel);
        powers.add((forward_power * heading_scale - angle_power) * vel);


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
        return powers;

    }

    public boolean isFinished() {
        return finished;
    }
}
