package org.firstinspires.ftc.teamcode.common.commandbase.subsytem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.ff.Alliance;

@Config
public class Turret extends SubsystemBase {
    private final Servo left;
    private final Servo right;

    public static double left_middle = 0.455;
    public static double right_middle = 0.49;


    public static double offset = 0.455;

    public Turret(Servo l, Servo r) {
        left = l;
        right = r;
    }

    public void middle() {
        left.setPosition(left_middle);
        right.setPosition(right_middle);
    }

    //actually goes to the right
    public void left() {
        left.setPosition(left_middle + offset);
        right.setPosition(right_middle + offset);
    }

    //actually goes to the left
    public void right() {
        left.setPosition(left_middle - offset);
        right.setPosition(right_middle - offset);
    }

    public void shared(Alliance alliance) {
        if (alliance == Alliance.RED) {
            right();
        } else {
            left();
        }
    }
}
