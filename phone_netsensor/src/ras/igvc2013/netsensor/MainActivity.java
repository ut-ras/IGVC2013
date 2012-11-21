package ras.igvc2013.netsensor;

import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.SystemClock;
import android.app.Activity;
import android.content.Context;
import android.view.Menu;
import android.view.ViewGroup.LayoutParams;
import android.widget.LinearLayout;
import android.widget.TextView;

public class MainActivity extends Activity {
	
	LinearLayout layout;
	
	SensorManager senManager;
	SensorEventListener magListener;
	SensorEventListener accelListener;
	
	LocationManager locManager;
	LocationListener locListener;
	
	DatagramSocket socket;
	SocketAddress address;
	ByteBuffer buffer;
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        
        layout = (LinearLayout)findViewById(R.id.lay);
        
        try {
        	// This needs to be replaced by the ip address 
        	// when doloras's network is constructed
			socket = new DatagramSocket(); //new Socket("doloras.kicks-ass.org", 11111)
			address = new InetSocketAddress("doloras.kicks-ass.org", 11111);
			
			buffer = ByteBuffer.allocate(32);
		} catch (IOException e) {
			e.printStackTrace();
		}
        
        locManager = (LocationManager)this.getSystemService(Context.LOCATION_SERVICE);
        locListener = new Loc(locManager);
		locManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locListener);
        
        senManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
        
		Sensor mag = senManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
		magListener = new Sen(mag, senManager, (byte)'m');
		senManager.registerListener(magListener, mag, SensorManager.SENSOR_DELAY_FASTEST);
		
		Sensor accel = senManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		accelListener = new Sen(accel, senManager, (byte)'a');
		senManager.registerListener(accelListener, accel, SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.activity_main, menu);
        return true;
    }
    
    @Override
    public void onPause() {
    	locManager.removeUpdates(locListener);
    	senManager.unregisterListener(magListener);
    	senManager.unregisterListener(accelListener);
    			
    	super.onPause();
    }
    
    public void send(ByteBuffer buffer) {
    	try {
			socket.send(new DatagramPacket(buffer.array(), buffer.limit(), address));
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
    
    private class Loc implements LocationListener {
    	private TextView[] gps = new TextView[3];
    	
    	public Loc(LocationManager manager) {
    		gps[0] = (TextView)findViewById(R.id.GPSX);
            gps[1] = (TextView)findViewById(R.id.GPSY);
            gps[2] = (TextView)findViewById(R.id.GPSZ);
            gps[0].setText("Latitude: Waiting . . .");
    		gps[1].setText("Longitude: Waiting . . .");
    		gps[2].setText("Altitude: Waiting . . .");
    	}
    	
    	public void onLocationChanged(Location loc) {
    		gps[0].setText("Latitude : " + Location.convert(loc.getLatitude() , Location.FORMAT_SECONDS));
    		gps[1].setText("Longitude: " + Location.convert(loc.getLongitude(), Location.FORMAT_SECONDS));
    		gps[2].setText("Altitude : " + Location.convert(loc.getAltitude() , Location.FORMAT_SECONDS));
    		
    		buffer.clear();
    		buffer.put((byte) 'g');
    		buffer.putDouble(loc.getLatitude());
    		buffer.putDouble(loc.getLongitude());
    		buffer.putDouble(loc.getAltitude());
    		buffer.flip();
    		
    		send(buffer);
    	}

		public void onProviderDisabled(String provider) {}
		public void onProviderEnabled(String provider) {}
		public void onStatusChanged(String provider, int status, Bundle extras) {}
    }
    
    private class Sen implements SensorEventListener {
    	private byte sig;
    	
    	private TextView name;
    	private TextView speed;
    	
    	private long lastTime;
    	private double avgTime;
    	
    	public Sen(Sensor s, SensorManager manager, byte sig) {
    		LayoutParams lparams = new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT);
    		
    		name = new TextView(MainActivity.this);
    		name.setLayoutParams(lparams);
    		name.setText(s.getName());
    		layout.addView(name);
    		
    		speed = new TextView(MainActivity.this);
    		speed.setLayoutParams(lparams);
    		speed.setText("speed: Waiting . . .");
    		layout.addView(speed);
    		
    		lastTime = SystemClock.uptimeMillis();
    		this.sig = sig;
    	}
    	
    	public void onSensorChanged(SensorEvent event) {
    		long temp = SystemClock.uptimeMillis();
    		avgTime = (0.99*avgTime) + (0.01*1000/(temp-lastTime));
    		if (temp != lastTime) speed.setText("speed: "  + (int)avgTime);
    		lastTime = temp;
    		
    		buffer.clear();
    		buffer.put(sig);
    		buffer.putFloat(event.values[0]);
    		buffer.putFloat(event.values[1]);
    		buffer.putFloat(event.values[2]);
    		buffer.flip();
    		
    		send(buffer);
		}
    	
		public void onAccuracyChanged(Sensor sensor, int accuracy) {}
    }
}
