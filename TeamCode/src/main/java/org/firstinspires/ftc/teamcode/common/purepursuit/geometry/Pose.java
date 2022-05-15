package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.geometry.Pose2d;

import java.util.Locale;

public class Pose extends Point {

    public double angle;

    public Pose(double x, double y, double angle) {
        super(x, y);
        this.angle = angle;
    }

    public Pose(Point o, double t) {
        this(o.x, o.y, t);
    }

    public Pose(double x, double y) {
        this(x, y, 0);
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(Pose other) {
        this(other.x, other.y, other.angle);
    }


    public Pose(Point point) {
        this(point.x, point.y);
    }

    public Pose(Pose2d pose2d) {
        this(pose2d.getX(), pose2d.getY(), pose2d.getRotation().getDegrees());
    }

    @NonNull
    public Pose clone() {
        return new Pose(this);
    }


    public Pose subtract(Pose o) {
        double diff = angle - o.angle;
        diff %= 360;
        if (diff > 180) diff -= 360;
        return new Pose(x - o.x, y - o.y, diff);
    }

    public double distanceTo(Point o) {
        return this.subtract(o).radius();
    }

    public Pose subtract(Point o) {
        return new Pose(x - o.x, y - o.y, angle);
    }

    public Pose scale(double scalar) {
        return new Pose(x * scalar, y * scalar, angle * scalar);
    }

    public Pose divideBy(Pose o) {
        return new Pose(x / o.x, y / o.y, angle / o.angle);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "(x: %.2f, y: %.2f)(r: %.2f, d: %.2f)", x, y, angle, Math.toDegrees(angle));
    }
}