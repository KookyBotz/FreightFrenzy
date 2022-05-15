package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

import java.util.Locale;

public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point rotated(double angle) {
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Point(newX, newY);
    }


    public Point add(Point p) {
        return new Point(this.x + p.x, this.y + p.y);
    }

    public Point subtract(Point p) {
        return new Point(this.x - p.x, this.y - p.y);
    }

    public double atan() {
        return Math.atan2(y, x);
    }

    public double radius() {
        return Math.sqrt(x * x + y * y);
    }

    public double distance(Point p) {
        return subtract(p).radius();
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "(%.1f, %.1f)", x, y);
    }
}