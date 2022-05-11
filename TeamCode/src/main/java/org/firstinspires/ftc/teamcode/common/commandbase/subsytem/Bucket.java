package org.firstinspires.ftc.teamcode.common.commandbase.subsytem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Bucket extends SubsystemBase {
    private final Servo dump;

    public static double in_position = 0.7;
    public static double rest_position = 0.6;
    public static double dump_position = 0.9;

    public Bucket(Servo d) {
        dump = d;
    }

    public void in() {
        dump.setPosition(in_position);
    }

    public void rest() {
        dump.setPosition(rest_position);
    }


    public void dump() {
        dump.setPosition(dump_position);
    }
}
