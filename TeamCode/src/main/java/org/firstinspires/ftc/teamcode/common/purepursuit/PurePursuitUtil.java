package org.firstinspires.ftc.teamcode.common.purepursuit;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Point;
import org.firstinspires.ftc.teamcode.common.purepursuit.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

@Config
public class PurePursuitUtil {
    public static double default_power = 0.2;
    public static double max_acceleration = 40;

    public static Pose P_coefficients = new Pose(24, 0, Math.PI/1.5);

    public static double admissible_error = 3;

    public static Point lineCircleIntersection(Point pointA, Point pointB, Point center, double radius) {
        double baX = pointB.x - pointA.x;
        double baY = pointB.y - pointA.y;
        double caX = center.x - pointA.x;
        double caY = center.y - pointA.y;
        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;
        double pBy2 = bBy2 / a;
        double q = c / a;
        double disc = pBy2 * pBy2 - q;
        if (disc < 0) {
            return pointA;
        }

        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;
        Point p1 = new Point(pointA.x - baX * abScalingFactor1, pointA.y - baY * abScalingFactor1);
        if (disc == 0) {
            return p1;
        }
        Point p2 = new Point(pointA.x - baX * abScalingFactor2, pointA.y - baY * abScalingFactor2);
        return Math.hypot(pointB.x - p1.x, pointB.y - p1.y) > Math.hypot(pointB.x - p2.x, pointB.y - p2.y) ? p2 : p1;
    }

    public static double getCurvature(double x, double l) {
        return (2 * x) / (l * l);
    }

    public static double getX(Pose robot, Point intersection) {
        double a = -Math.tan(Math.toRadians(robot.angle));
        double b = 1;
        double c = Math.tan(Math.toRadians(robot.angle)) * robot.x - robot.y;

        return Math.abs(a * intersection.x + b * intersection.y + c) / Math.sqrt(a * a + b * b);
    }

    public static double getSign(Pose robot, Point intersection) {
        return -Math.signum(Math.sin(Math.toRadians(robot.angle)) * (intersection.x - robot.x) - Math.cos(Math.toRadians(robot.angle)) * (intersection.y - robot.y));
    }

    public static List<Double> calculateWheelSpeeds(Pose robot, Point intersection, double curvature, double sign, double target_velocity, double track_width) {
        double d = robot.distanceTo(intersection);
        double arc_radius = 1 / curvature;

        double theta = Math.acos(1 - ((d * d) / (2 * arc_radius * arc_radius)));

        double left_radius = arc_radius + track_width / 2;
        double right_radius = arc_radius - track_width / 2;
        double left_arc = left_radius * theta;
        double right_arc = right_radius * theta;
        double max = Math.max(Math.abs(left_arc), Math.abs(right_arc));

        left_arc /= max;
        right_arc /= max;

        left_arc *= target_velocity;
        right_arc *= target_velocity;

        List<Double> powers = new ArrayList<>();
        if (sign == -1) {
            powers.add(left_arc);
            powers.add(right_arc);
        } else {
            powers.add(right_arc);
            powers.add(left_arc);
        }

        return powers;
    }
}
