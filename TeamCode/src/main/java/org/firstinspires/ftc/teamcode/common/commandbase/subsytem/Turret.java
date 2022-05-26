package org.firstinspires.ftc.teamcode.common.commandbase.subsytem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Turret extends SubsystemBase {
    private final Servo left;
    private final Servo right;

    public static double left_middle = 0.465;
    public static double right_middle = 0.485;


    public static double offset = 0.465;

    public Turret(Servo l, Servo r) {
        left = l;
        right = r;
    }

    public void middle() {
        left.setPosition(left_middle);
        right.setPosition(right_middle);
    }

    public void left() {
        left.setPosition(left_middle + offset);
        right.setPosition(right_middle + offset);
    }

    public void right() {
        left.setPosition(left_middle - offset);
        right.setPosition(right_middle - offset);
    }

    public void turn(int pos) {
        if (pos == 0) {
            middle();
        } else if (pos == 1) {
            right();
        } else {
            left();
        }
    }
}
