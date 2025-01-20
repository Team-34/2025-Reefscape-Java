package frc.robot;

import com.studica.frc.AHRS;

public class Gyro extends AHRS {
    private static final Gyro instance = new Gyro();

    private Gyro() {
        super(NavXComType.kMXP_SPI);
    }

    public static Gyro get() {
        return instance;
    }

    public void zeroYaw() {
        this.reset();
    }
}
