package org.firstinspires.ftc.teamcode.common.purepursuit.geometry;

import java.util.Locale;

public class Waypoint extends Point {
    public final double radius;
    public boolean reversed;

    public Waypoint(double x, double y, double r) {
        super(x, y);
        radius = r;
        this.reversed = false;
    }

    public Waypoint(double x, double y, double r, boolean reversed){
        this(x, y, r);
        this.reversed = reversed;
    }

    public Point getPoint(){
        return new Point(x, y);
    }

    @Override
    public String toString() {
        return super.toString().concat(String.format(Locale.ENGLISH, "(r: %.2f)", radius));
    }
}
