import java.io.IOException;

import org.json.JSONException;
import org.json.JSONObject;

import edu.brown.robotics.rosbridge.MessageHandler;
import edu.brown.robotics.rosbridge.Rosbridge;


public class Test {

	/**
	 * @param args
	 * @throws JSONException 
	 * @throws IOException 
	 */
	public static void main(String[] args) throws JSONException, IOException {
		
		MessageHandler handler = new MessageHandler() {

			@Override
			public void messageReceived(JSONObject message) {
				System.out.println("Received message" + message.toString());
			}};
		
		Rosbridge bridge = new Rosbridge("localhost", 9090);
		bridge.publish("/jon", "std_msgs/String", strmessage("Hello"));
		bridge.subscribe("/jon", handler, "std_msgs/String");
		while (true) {
			System.out.println("Processing incoming");
			bridge.processIncomingMessages(true);
		}
	}
	
	private static JSONObject strmessage(String msg) throws JSONException {
		JSONObject message = new JSONObject();
		message.put("data", msg);
		return message;
	}

}
