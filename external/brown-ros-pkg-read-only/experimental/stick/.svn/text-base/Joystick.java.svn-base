import java.applet.Applet;
import java.net.Socket;
import java.io.OutputStream;

public class Joystick extends Applet {
	Socket socket = null;
	OutputStream out = null;

	public void stop() {
		if (socket != null) {
			try {
				socket.close();
			} catch(java.io.IOException ioe) {
				//pass
			}
		}
	}

	public void destroy() {
		stop();
	}

	public void connect(String host, int port) {
		stop();

		try {
			socket = new java.net.Socket(host,port);
		} catch (java.net.UnknownHostException uhe) {
			//pass
		} catch (java.io.IOException ioe) {
			//pass
		}

		if (socket == null) {
			return;
		}

		try {
			out = socket.getOutputStream();
		} catch (java.io.IOException ioe) {
			//pass
		}

	}

	public void report(int x, int z) {
		if (out == null) {
			return;
		}

		byte data[] = new byte[2];
		data[0] = (byte) x;
		data[1] = (byte) z;
		
		try {
			out.write(data);
			out.flush();
		} catch (java.io.IOException ioe) {
			//pass
		}

	}
}
