package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * interface for hardware components
 * @author Anthony Rubick
 */
public interface Component {
    //enforce that every one of these has an innit method
    void init(HardwareMap ahwMap);

    /**
     * @return a list of all hardware devices included on the component
     */
    List<HardwareDevice> getAll();
}
