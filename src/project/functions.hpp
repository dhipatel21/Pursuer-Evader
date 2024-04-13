#include <cstdlib>
#include <string>
#include <iostream>

#define VENDOR_ID 0x2752
#define PRODUCT_ID 0x1C

int listen(unsigned char* data);

void playWav(const std::string& filePath);