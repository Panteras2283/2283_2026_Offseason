package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.net.HttpURLConnection;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

public class LEDs extends SubsystemBase {
  // Keeps track of the currently active preset to prevent spamming
  private int currentPreset = -1;

  public LEDs() {}

  @Override
  public void periodic() {}

  /**
   * Helper method to handle asynchronous HTTP requests and state tracking.
   */
  private void setPreset(int presetId) {
    // 1. Edge Detection: Do nothing if the requested preset is already active
    if (currentPreset == presetId) {
      return;
    }
    currentPreset = presetId;

    // 2. Asynchronous Execution: Run the network request on a background thread
    CompletableFuture.runAsync(() -> {
      try {
        String targetUrl = "http://10.22.83.100/win&PL=" + presetId;
        URL url = new URL(targetUrl);
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("GET");
        
        // Add timeouts so a disconnected LED controller doesn't hang the background thread
        connection.setConnectTimeout(500); 
        connection.setReadTimeout(500);

        // We only need the response code to trigger the change, no need to read the whole body
        int responseCode = connection.getResponseCode();
        connection.disconnect();

      } catch (Exception e) {
        System.out.println("LED HTTP Request failed: " + e.getMessage());
        // Reset the state so it attempts to send the request again next loop
        currentPreset = -1; 
      }
    });
  }

  public void Default() {
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent() && alliance.get() == Alliance.Blue){
      setPreset(1);
    } else if(alliance.isPresent() && alliance.get() == Alliance.Red){
      setPreset(2);
    }
  }

  public void Feed() {
    setPreset(3);
  }

  public void Idle() {
    setPreset(4);
  }

  public void RTF() {
    setPreset(5);
  }

  public void Antijam(){
    setPreset(6);
  }
}