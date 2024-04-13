#include <iostream>
#include <libusb-1.0/libusb.h>
#include "functions.hpp"

int listen(unsigned char* data) {   // input to function is a 6 byte unsigned char array --> unsigned char data[6]
    libusb_context* context = NULL;
    libusb_device_handle* deviceHandle = NULL;

    // Initialize libusb
    int result = libusb_init(&context);
    if (result < 0) {
        std::cerr << "Error initializing libusb\n";
        return 1;
    }

    // Find the USB device
    deviceHandle = libusb_open_device_with_vid_pid(context, VENDOR_ID, PRODUCT_ID);
    if (deviceHandle == NULL) {
        std::cerr << "Error finding USB device\n";
        libusb_exit(context);
        return 1;
    }

    // Claim the interface
    result = libusb_claim_interface(deviceHandle, 0);
    if (result < 0) {
        std::cerr << "Error claiming USB interface\n";
        libusb_close(deviceHandle);
        libusb_exit(context);
        return 1;
    }

    // Receive interrupts
    int bytesRead;
    result = libusb_interrupt_transfer(deviceHandle, 0x81, data, sizeof(data), &bytesRead, 1000); // Endpoint address may vary
    if (result == 0) {
        // Process received data
        // For example:
        std::cout << "Received command: ";
        for (int i = 0; i < bytesRead; ++i) {
            std::cout << std::hex << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::endl;
    } else {
        std::cerr << "Error receiving interrupt: " << libusb_error_name(result) << std::endl;
    }

    // Release the interface and cleanup
    libusb_release_interface(deviceHandle, 0);
    libusb_close(deviceHandle);
    libusb_exit(context);

    return 0;
}