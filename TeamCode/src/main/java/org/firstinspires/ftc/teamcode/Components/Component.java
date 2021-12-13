package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Component {
    //enforce that every one of these has an innit method
    void init(HardwareMap ahwMap);
}
