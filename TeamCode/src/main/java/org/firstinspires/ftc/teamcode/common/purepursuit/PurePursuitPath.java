package org.firstinspires.ftc.teamcode.common.purepursuit;

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

    public PurePursuitPath(Waypoint... w) {
        waypoints = new LinkedList<>(Arrays.asList(w));

        if (waypoints.size() < 2) throw new IllegalArgumentException("Bro....");
    }

    public List<Double> update(Pose robot) {
        if (finished) return new ArrayList<>(Collections.nCopies(2, 0.0));

        Waypoint a = waypoints.get(current);
        Waypoint b = waypoints.get(current + 1);
        double radius = b.radius;

        if (current == waypoints.size() - 2 && robot.distanceTo(b) < PurePursuitUtil.admissible_error) {
            finished = true;
            return new ArrayList<>(Collections.nCopies(2, 0.0));
        } else if (current == waypoints.size() - 2 && robot.distanceTo(b) < radius) {
//            double angle = Math.toDegrees(b.subtract(robot).atan());
//            double reverse_angle = Math.toDegrees(Math.toRadians(angle + 180));
//            double auto_angle = Math.abs(robot.angle - angle) <
//                    Math.abs(robot.angle - reverse_angle) ?
//                    angle : reverse_angle;
//
//            double angle_power = auto_angle / PurePursuit.P_coefficients.angle;
//            double controlled_power = robot.distanceTo(b) / PurePursuit.P_coefficients.x;
//
//
//            ArrayList<Double> powers = new ArrayList<>();
//            powers.add(controlled_power + angle_power);
//            powers.add(controlled_power - angle_power);
//
//            return powers;
            finished = true;
            return new ArrayList<>(Collections.nCopies(2, 0.0));
        } else if (robot.distanceTo(b) < radius) {
            current++;
            a = waypoints.get(current);
            b = waypoints.get(current + 1);
            radius = b.radius;
        }

        // handle acceleration
        if (current == 0) {

        }

        Point target = PurePursuitUtil.lineCircleIntersection(a, b, robot, radius);

        double angle = Math.toDegrees(target.subtract(robot).atan()) - robot.angle;
        //double reverse_angle = Math.toDegrees(Math.toRadians(angle + 180));
        //double auto_angle = Math.abs(robot.angle - angle) <
        //        Math.abs(robot.angle - reverse_angle) ?
        //        angle : reverse_angle;

        double angle_power = angle / PurePursuitUtil.P_coefficients.angle;
        double forward_power = robot.distanceTo(target) / PurePursuitUtil.P_coefficients.x;

        double heading_scale = Math.abs(Math.cos(Math.min(Math.max(-Math.PI / 2, angle - robot.angle), Math.PI / 2)));

        ArrayList<Double> powers = new ArrayList<>();
        powers.add(forward_power * heading_scale + angle_power);
        powers.add(forward_power * heading_scale - angle_power);

        return powers;
    }

    public boolean isFinished() {
        return finished;
    }
}
