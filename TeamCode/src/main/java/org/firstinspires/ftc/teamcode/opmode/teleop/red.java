package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.ff.Alliance;

@TeleOp
public class red extends opmode {
    @Override
    public void initialize(){
        super.initialize();
        this.alliance = Alliance.RED;
    }
}
