package org.firstinspires.ftc.teamcode.managers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputManager {
	boolean isInit = false;    
	private Gamepad gamepad1;
	private Gamepad gamepad2;
	// Bill Pugh Singleton Code
    // ================================================ 
		private InputManager() {}
		
		private static class Holder {
			private static final InputManager INSTANCE = new InputManager();
		}
		
		public static InputManager getInstance() {
			return Holder.INSTANCE;
		}
	// ================================================

    // Call in OpMode init phase to set gamepad varaibles
    public void init(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2; 
		isInit = true;
    }


	public Gamepad getGamepad1(){
		if(isInit){
			return gamepad1;
		}
		else{
			// apparently you can put a backslash to split into multiple lines,
			// and i am too lazy to fix it, 
			// so this is jst going to wrap around. Sorry not sorry
			throw new RuntimeException("Error: Attempted to access uninitialized Singleton! \n File: managers, in getGamepad1");
		}
	}


	public Gamepad getGamepad2(){
		if(isInit){
			return gamepad2;
		}
		else{
			throw new RuntimeException("Error: Attempted to access uninitialized Singleton! \n File: managers, in getGamepad2");
		}
	}


}
