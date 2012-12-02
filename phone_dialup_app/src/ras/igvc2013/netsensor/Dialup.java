package ras.igvc2013.netsensor;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioTrack;
import android.os.AsyncTask;

public class Dialup extends AsyncTask<Void, Void, Void> {
	public int sampleRate = 44100;
	public int blockSize;
	private final int bytelen = 60;
	
	private AudioTrack outSignal;
	
	private BlockingQueue<byte[]> buffer;
	
	public Dialup() {
		blockSize = 0;
	}
	
	public Dialup(int msglen) {
		blockSize = msglen;
	}
	
	@Override
	protected void onPreExecute() {
		
		buffer = new ArrayBlockingQueue<byte[]>(4);
		
		int min = AudioTrack.getMinBufferSize(sampleRate, AudioFormat.CHANNEL_CONFIGURATION_MONO, AudioFormat.ENCODING_PCM_8BIT);
		min = bytelen * ((min / bytelen) + 1);
		
		if (min > blockSize)
			blockSize = min;
		
		outSignal = new AudioTrack( 
				AudioManager.STREAM_MUSIC, sampleRate, 
			    AudioFormat.CHANNEL_OUT_MONO, 
			    AudioFormat.ENCODING_PCM_8BIT, 
			    blockSize,
			    AudioTrack.MODE_STREAM
		);
		outSignal.play();
	}

	@Override
	protected Void doInBackground(Void... params) {
		try {
			byte[] block = new byte[blockSize];
			
			while (!this.isCancelled()) {
				// block if no data is available
				byte[] data = buffer.take();
				int wordCount = 0;
				
				while (wordCount < data.length) {
					for (int blockCount = 0; blockCount < blockSize/bytelen; blockCount++) {
						byte word = data[wordCount++];
						
						// Invert data due to natural inversion of the signal
						for (int j=0; j<8; j++) { 
							// create either the 1 or 0 signal
							block[blockCount*bytelen + j*6 + 0] = 
							block[blockCount*bytelen + j*6 + 1] = 0;
							block[blockCount*bytelen + j*6 + 2] = 
							block[blockCount*bytelen + j*6 + 3] = (byte) ((((word >> 7-j) & 1) ^ 1) * Byte.MAX_VALUE);
							block[blockCount*bytelen + j*6 + 4] = 
							block[blockCount*bytelen + j*6 + 5] = Byte.MAX_VALUE;
						}
						
						// End of word or end of message byte
						block[blockCount*bytelen + 48 + 0] = 
						block[blockCount*bytelen + 48 + 1] = 0;
						block[blockCount*bytelen + 48 + 2] = 
						block[blockCount*bytelen + 48 + 3] =
						block[blockCount*bytelen + 48 + 4] = 
						block[blockCount*bytelen + 48 + 5] =
						block[blockCount*bytelen + 48 + 6] = 
						block[blockCount*bytelen + 48 + 7] =
						block[blockCount*bytelen + 48 + 8] = 
						block[blockCount*bytelen + 48 + 9] = wordCount < data.length ? 0 : Byte.MAX_VALUE;
						block[blockCount*bytelen + 48 +10] = 
						block[blockCount*bytelen + 48 +11] = Byte.MAX_VALUE;
						
						if (wordCount >= data.length)
							break;
					}
					
					outSignal.write(block, 0, wordCount*bytelen);
				}
			}
		} catch (InterruptedException e) {}
		
		return null;
	}
	
	public synchronized boolean send(byte[] data) {
		byte[] holder = new byte[data.length + 1];
		System.arraycopy(data, 0, holder, 0, data.length);
		
		for (byte b : data) {
			holder[holder.length-1] ^= b;
		}
			
		// discard packet if buffer is full
		return buffer.offer(holder);
	}
}
