package frc.robot.subsystems.indication;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Viewport extends SubsystemBase {

    private int port;
    
    public Viewport(int port) {
        this.port = port;

        UsbCamera camera = CameraServer.startAutomaticCapture(port);
        camera.setFPS(15);
        // camera.setResolution(..., ...)
    }
}

// http://roboRIO-2990-frc.local:1181