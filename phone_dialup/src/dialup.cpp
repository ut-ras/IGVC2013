#include "dialup.h"

#include <exception>
#include <pulse/error.h>

#include <cstdio>

class pa_exception: public std::exception {
public:
    int error;

    pa_exception(int e) { error = e; }
    const char *what() const throw() { return pa_strerror(error); }
};


Dialup::Dialup(int len) {
    buffsize = len;
    buffind = 0;
    buffer = new unsigned char[len];

    int err;
    pa_sample_spec spec;
    pa_buffer_attr bfat;

    bfat.maxlength = -1;
    bfat.fragsize = -1;

    spec.format = PA_SAMPLE_U8;
    spec.channels = 2;
    spec.rate = 44100;

    

    audiosrv = pa_simple_new(
        0, // default pulseaudio server
        "phone_dialup", // app name
        PA_STREAM_RECORD,
        0, // default device
        "phone dialup rosnode", //description
        &spec,
        0, // default channel map
        &bfat,
        &err  // get error codes
    );

    if (!audiosrv) throw pa_exception(err);
}

Dialup::~Dialup() {
    delete [] buffer;
    pa_simple_free(audiosrv);
}


// Reads in a message and places it in out limited to the len given
// returns the size of the message
size_t Dialup::read(unsigned char *out, size_t len) {
    // TODO: maybe clean it up a bit more, 
    // it my be losing some bits when it refills the buffer
    int high = 0;
    int low = 0;

    unsigned char hash = 0;

    size_t dataind = 0;
    buffer[dataind] = 0;

    // bypass everthing that isn't the signal
    while (!buffer[buffind]) buffind++;

    while (dataind < len) {
        // grab new data if were done with this block
        if (buffind > buffsize) {
//            printf("!");

            // save the last value for the case of a floating value
            int oldval = buffer[buffsize-2];
            buffind = 0;

            // get the next block of data
            int err = pa_simple_read(audiosrv, buffer, buffsize, 0);
            if (err) throw pa_exception(err);

            // flip the data upside down
            // if its in a middle range assume the previous value
            for (int i=0; i<buffsize; i++) {
                if (buffer[i] < 100) {
                    buffer[i] = 0;
                } else if (buffer[i] > 150) {
                    buffer[i] = 1;
                } else {
                    buffer[i] = i>0 ? buffer[i-1] : oldval;
                }
            }
        }


        // count up highs and lows
        if (buffer[buffind])
            high++;
        else
            low++;

        if (low > 8) { // End of message signal
            if (out[dataind] != hash)
                return -1;
            else
                return dataind;
        }


        if (low+high > 4 && low && buffer[buffind]) {
            if (high > 8) { // End of word signal
//                printf(" |%d|%d[%c]\n", high, low, out[dataind]);

                hash ^= out[dataind];
                out[++dataind] = 0;
            } else { // determine the bit
//                printf("%d", high>low);

                out[dataind] <<= 1;
                out[dataind] |= (high > low);
            }

            high = 0;
            low = 0;
        }

        //skip two as the audio data is interleaved
        buffind += 2;
    }

    return -2;
}
