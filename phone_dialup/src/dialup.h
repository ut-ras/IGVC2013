#ifndef DIALUP_H
#define DIALUP_H

#include <pulse/simple.h>

class Dialup {
private:
    size_t buffsize;
    size_t buffind;
    unsigned char *buffer;

    pa_simple *audiosrv;

public:
    Dialup(int size);
    ~Dialup();

    size_t read(unsigned char *buff, size_t len);
};

#endif
