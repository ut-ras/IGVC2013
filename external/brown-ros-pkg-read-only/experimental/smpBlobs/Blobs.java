import java.applet.Applet;
import java.net.Socket;
import java.io.OutputStream;
import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.util.Timer;
import java.util.TimerTask;

public class Blobs extends Applet {
	Socket socket = null;
	OutputStream out = null;
	BufferedReader in = null;
	String blobs = null;
	Timer monitor = null;

	public void start() {
		
	}

	public void stop() {
		if (socket != null) {
			try {
				socket.close();
			} catch(java.io.IOException ioe) {
				//pass
			}
		}

		if (monitor != null) {
			monitor.cancel();
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

		try {
			in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
		} catch (java.io.IOException ioe) {
			//pass
		}

		class Monitor extends TimerTask {
			Blobs blobs;

			public Monitor(Blobs blbs) {
				blobs = blbs;
			}

			public void run() {
				if (out == null) return;

				try {
					out.write('a');
					out.flush();
				} catch (java.io.IOException ioe) {
					return;
				}

				if (in == null) return;

				try {
					String line = in.readLine();
					if (line != null) {
						blobs.blobs = line;
					}
				} catch(java.io.IOException ioe) {
					//pass
				}
			}
		}

		monitor = new Timer();	
		monitor.schedule(new Monitor(this),33,33);

	}

	public String report() {
		if (blobs == null) {
			return "[[0,0,0]];";
		}
		return blobs;
	}

	/*
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
	*/

}
