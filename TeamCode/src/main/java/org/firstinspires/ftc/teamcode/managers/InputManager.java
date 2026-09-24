package org.firstinspires.ftc.teamcode.managers;
public class InputManager {
    // Bill Pugh Singleton Code
    private InputManager() {}
    private static class Holder {
        private static final InputManager INSTANCE = new InputManager();
    }
    public static InputManager getInstance() {
        return Holder.INSTANCE
    }
    // Gamepad object holder
    public Gamepad driverGamepad;
    // Call in OpMode init phase to set gamepad varaibles
    public void init(Gamepad gamepad1, Gamepad gamepad2) {
        this.driverGamepad = gamepad1;
    }
}
