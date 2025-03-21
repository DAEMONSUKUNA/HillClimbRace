package com.mygdx.hillclimbracing;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;

public class DesktopLauncher {
    public static void main(String[] arg) {
        Lwjgl3ApplicationConfiguration config = new Lwjgl3ApplicationConfiguration();
        config.setTitle("Hill Climb Racing"); // Set window title
        config.setFullscreenMode(Lwjgl3ApplicationConfiguration.getDisplayMode()); // Set fullscreen
        config.useVsync(true); // Enable vsync
        config.setForegroundFPS(60); // Limit FPS to 60
        new Lwjgl3Application(new HillClimbGame(), config);
    }
}
