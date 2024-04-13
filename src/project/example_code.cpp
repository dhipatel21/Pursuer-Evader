#include "functions.hpp"

// example code of how to get mic position and angle of direction number specified by mic position
int main() {
    unsigned char data[6];
    if (!listen(data) &&  data[2]) {    //successfully listened and VAD = 1 (voice detected)
        int dir = data[5];  // value ranges from 1-6 corresponding to mic position
        int angle = (data[3] << 8) | data[4];   // combining high byte and low byte into one to get angle (0-360)
    }
}