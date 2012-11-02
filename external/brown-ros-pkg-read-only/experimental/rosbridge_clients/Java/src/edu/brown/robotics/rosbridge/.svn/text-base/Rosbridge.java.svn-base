package edu.brown.robotics.rosbridge;

import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import java.util.HashMap;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class Rosbridge {
	
	public boolean open = true;
	
	private RosbridgeSocket rosbridgeSocket;
	
	private Map<String, Collection<MessageHandler>> handlers = new HashMap<String, Collection<MessageHandler>>();

	/**
	 * Creates a Rosbridge instance and connects to the specified hostname/port
	 * @param hostname
	 * @param port
	 * @throws IOException Propagates exceptions thrown by the Socket initialization
	 * @throws UnknownHostException Propagates exceptions thrown by the Socket initialization
	 */
	public Rosbridge(String hostname, int port) throws UnknownHostException, IOException {
		// Create the rosbridge socket
		this.rosbridgeSocket = new RosbridgeSocket(hostname, port);
	}
	
	/**
	 * Returns the underlying socket that this rosbridge is using.
	 * If you are selecting over sockets you will need to retrieve the socket using this method
	 * @return
	 */
	public Socket socket() {
		return this.rosbridgeSocket.socket;
	}
	
	/**
	 * Reads data from the socket, processes the Strings, then calls relevant callbacks
	 * on the data.  You can specify whether to block waiting for incoming data on the socket,
	 * or just to query the socket and return regardless.  It can be called from an event loop,
	 * or, you can select on the socket (returned by the socket() method) and call this
	 * method only when incoming data is received on the socket.
	 */
	public void processIncomingMessages(boolean block) {
		int bytesRead = this.rosbridgeSocket.read(block);
		while (this.rosbridgeSocket.hasMessages()) {
			String message = this.rosbridgeSocket.getMessage();
			try {
				JSONObject object = new JSONObject(message);
				String receiver = object.getString("receiver");
				this.callHandlers(receiver, object);
			} catch (JSONException e) {
				// Do nothing, treat as a bad message
				System.err.println("Unabled to deserialise message to JSON: " + message);
			}
		}
		if (bytesRead==-1) {
			this.open = false;
		}
	}
	
	/**
	 * Publishes the provided message on the specified topic.  
	 * @param topic The name of the topic to publish on
	 * @param type The message object to publish
	 * @param message The JSON object of the message
	 * @throws JSONException If the JSON library is unhappy with any of the provided arguments, the
	 * exception is propagated
	 * @throws IOException 
	 */
	public void publish(String topic, String type, JSONObject message) throws JSONException {
		JSONObject call = new JSONObject();
		call.put("receiver", topic);
		call.put("msg", message);
		call.put("type", type);
		sendMessage(call);
	}
	
	/**
	 * Calls the specified service with the given arguments.  When a response is received for the service,
	 * the response handler will be called with the service response.
	 * @param serviceName The name of the service to be called
	 * @param serviceArguments An array of service arguments 
	 * @param responseHandler The callback to be called upon service completion
	 * @throws JSONException If the JSON library is unhappy with any of the provided arguments, the
	 * exception is propagated
	 * @throws IOException 
	 */
	public void callService(String serviceName, JSONArray serviceArguments, MessageHandler responseHandler) {
		try {
			JSONObject call = new JSONObject();
			call.put("receiver", serviceName);
			call.put("msg", serviceArguments);
			sendMessage(call);
		} catch (JSONException e) {
			System.err.println("Unable to create JSON object for service invocation " + serviceName);
		}
	}
	
	/**
	 * Subscribes to the specified topic, using the specified handler as the callback handler
	 * Subsequent calls to processIncomingMessages will invoke the handler if a message arrives for that
	 * handler
	 * @param topic The topic to subscribe to
	 * @param handler The handler to register as a callback for new messages
	 */
	public void subscribe(String topic, MessageHandler handler) {
		this.subscribe(topic, handler, -1);
	}
	
	/**
	 * Subscribes to the specified topic, using the specified handler as the callback handler
	 * Subsequent calls to processIncomingMessages will invoke the handler if a message arrives for that
	 * handler
	 * @param topic The topic to subscribe to
	 * @param handler The handler to register as a callback for new messages
	 * @param messageRate The least amount of time to wait between messages, or -1 for as fast as possible
	 */
	public void subscribe(String topic, MessageHandler handler, int messageRate) {
		this.addHandler(topic, handler);
		JSONArray args = new JSONArray();
		args.put(topic);
		args.put(messageRate);
		this.callService("/rosbridge/subscribe", args, handler);
	}
	
	/**
	 * Subscribes to the specified topic, using the specified handler as the callback handler
	 * Subsequent calls to processIncomingMessages will invoke the handler if a message arrives for that
	 * handler
	 * @param topic The topic to subscribe to
	 * @param handler The handler to register as a callback for new messages
	 * @param typeHint The expected type of the topic. Required if the topic isn't yet advertised
	 */
	public void subscribe(String topic, MessageHandler handler, String typeHint) {
		this.subscribe(topic, handler, -1, typeHint);
	}
	
	/**
	 * Subscribes to the specified topic, using the specified handler as the callback handler
	 * Subsequent calls to processIncomingMessages will invoke the handler if a message arrives for that
	 * handler
	 * @param topic The topic to subscribe to
	 * @param handler The handler to register as a callback for new messages
	 * @param messageRate The least amount of time to wait between messages, or -1 for as fast as possible
	 * @param typeHint The expected type of the topic. Required if the topic isn't yet advertised
	 */
	public void subscribe(String topic, MessageHandler handler, int messageRate, String typeHint) {
		this.addHandler(topic, handler);
		JSONArray args = new JSONArray();
		args.put(topic);
		args.put(messageRate);
		args.put(typeHint);
		this.callService("/rosbridge/subscribe", args, handler);
	}

	/**
	 * calls tostring on the message, adds the start and end terminators, and sends on the socket.
	 * @throws IOException 
	 */
	private void sendMessage(JSONObject messageObject) {
		boolean success = this.rosbridgeSocket.sendMessage(messageObject.toString());
		this.open = success;
	}
	
	private void addHandler(String topic, MessageHandler handler) {
		Collection<MessageHandler> topichandlers = this.handlers.get(topic);
		if (topichandlers==null) {
			topichandlers = new ArrayList<MessageHandler>();
		}
		topichandlers.add(handler);
		this.handlers.put(topic, topichandlers);		
	}

	private void callHandlers(String topic, JSONObject message) {
		Collection<MessageHandler> topichandlers = handlers.get(topic);
		if (topichandlers==null) {
			return;
		}
		for (MessageHandler handler : topichandlers) {
			try {
				handler.messageReceived(message);
			} catch(Exception e) {
				System.err.println("Exception in callback handler for topic " + topic);
			}
		}
	}

}
