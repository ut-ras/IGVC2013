package edu.brown.robotics.rosbridge;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;

public class RosbridgeSocket {
	
	private static byte prefix = 0;
	private static byte suffix = -1;
	
	public Socket socket;
	
	private OutputStream output;
	private InputStream input;
	
	private byte[] incomingData = new byte[0];
	private List<String> incomingMessages = new ArrayList<String>();
	
	/**
	 * Creates a Rosbridge instance and connects to the specified hostname/port
	 * @param hostname
	 * @param port
	 * @throws IOException Propagates exceptions thrown by the Socket initialization
	 * @throws UnknownHostException Propagates exceptions thrown by the Socket initialization
	 */
	public RosbridgeSocket(String hostname, int port) throws UnknownHostException, IOException {
		// Create the socket
		this.socket = new Socket(hostname, port);
		
		// Create the input and output writers
		this.output = this.socket.getOutputStream();
		this.input = this.socket.getInputStream();
		
		// Write the handshake
		this.output.write("raw\r\n\r\n".getBytes("ASCII"));
		output.flush();
	}
	
	/**
	 * Sends the specified message across the raw rosbridge connection.
	 * @param message The string message to send
	 * @return true if the message was succesfully sent, false otherwise
	 */
	public boolean sendMessage(String message) {
		try {
			// Extract the bytes from the message
			byte[] messageBytes = message.getBytes("ASCII");
			
			// Create the bytes with prefix and suffix
			byte[] fullMessage = new byte[messageBytes.length+2];
			System.arraycopy(messageBytes, 0, fullMessage, 1, messageBytes.length);
			fullMessage[0] = prefix;
			fullMessage[messageBytes.length+1] = suffix;
			
			// Send the new message
			output.write(fullMessage);
			output.flush();
			
			return true;
		} catch (IOException e) {
			System.err.println("IOException writing "+message.toString()+" to socket.  " +
					"Socket may be closed");
			e.printStackTrace();
			return false;
		}
	}
	
	/**
	 * Reads from the socket.  Returns the number of bytes read, or -1 if socket closed.
	 * User can specify whether to block, waiting for incoming data, or just to query the
	 * stream and return immediately if no pending data.
	 * After calling this method, call hasMessages to see whether there are now queued
	 * messages ready for delivery.
	 * @param block if true, the method will block until at least some data has been read
	 * from the socket
	 * @return the number of bytes read from the socket
	 * @throws IOException
	 */
	public int read(boolean block) {
		int numBytes = 0;
		try {
			if (this.input.available()>0 || block) {
				numBytes = readBytes();
			}
		} catch (IOException e) {
			// IOException thrown if the socket is closed. Return -1 to indicate thus
			return -1;
		}
		
		parseMessages();

		return numBytes;
	}
	
	public boolean hasMessages() {
		return this.incomingMessages.size()>0;
	}
	
	public String getMessage() {
		return this.incomingMessages.remove(0);
	}
	
	/**
	 * Reads bytes from the input stream and adds them to the incomingData buffer
	 * @return -1 if -1 returned from read call, otherwise num bytes read
	 * @throws IOException 
	 */
	private int readBytes() {
		// Read the incoming data
		byte[] inputBuffer = new byte[65535];
		int read;
		try {
			read = this.input.read(inputBuffer, 0, inputBuffer.length);
		} catch (IOException e) {
			// Just close the socket and return
			return -1;
		}
		if (read==-1) {
			return -1;
		}
		
		// Add read data to the incoming bytes buffer
		byte[] bytesBuffer = new byte[read + incomingData.length];
		System.arraycopy(incomingData, 0, bytesBuffer, 0, incomingData.length);
		System.arraycopy(inputBuffer, 0, bytesBuffer, incomingData.length, read);
		
		this.incomingData = bytesBuffer;
		
		return read;
	}
	
	/**
	 * Processes the incomingData buffer, turning raw messages into strings and adding 
	 * to the incomingMessages queue.
	 */
	private void parseMessages() {
		boolean parsed = true;
		while (parsed) {
			parsed = parseMessage();
		}
	}
	
	/**
	 * Parses the data in the incomingData buffer, reduces the buffer size
	 * @return true if a message was parsed, false otherwise
	 */
	private boolean parseMessage() {
		// Find the position of the start and stop indicators
		int start = messageStartIndex();
		int stop = messageStopIndex();
		
		if (start!=-1 && stop!=-1 && stop>start) {
			// If there is a message here, copy out the message and update buffer
			byte[] message = new byte[stop-start-1];
			System.arraycopy(incomingData, start+1, message, 0, stop-start-1);
			try {
				incomingMessages.add(new String(message, "ASCII"));
			} catch (UnsupportedEncodingException e) {
			}
			
			cutHead(stop+1);
			
			return true;
		} else if (start>0) {
			cutHead(start);
		} else if (start==-1){
			incomingData = new byte[0];
		}
		
		return false;
	}
	
	private void cutHead(int start) {
		byte[] newData = new byte[incomingData.length - start];
		System.arraycopy(incomingData, start, newData, 0, incomingData.length-start);
		incomingData = newData;
	}
	
	private int messageStartIndex() {
		for (int i = 0; i < incomingData.length; i++) {
			if (incomingData[i]==prefix) {
				return i;
			}
		}
		return -1;
	}
	
	private int messageStopIndex() {
		for (int i = 0; i < incomingData.length; i++) {
			if (incomingData[i]==suffix) {
				return i;
			}
		}
		return -1;
	}
	
	
}
