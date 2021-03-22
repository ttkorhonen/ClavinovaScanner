# ClavinovaScanner
Teensy 4.1 sketch to scan Yamaha Clavinova digital piano keyboard and pedals
============================================================================

The Arduino/Teensyduino sketch scans the 88-key keyboard of a Yamaha Clavinova CVP-65 and sends
corresponding MIDI messages to a USB host.
The sketch supports velocity and three pedals (sustain, sostenuto and soft).
The scanning of the 12*15 matrix of the Clavinova is done "brute force", just waiting for key chatter to settle
and then reading all inputs one by one. 

One full cycle on a Teensy 4.1 takes about 370 us, which
is more than fast enough. Further optimization may be done in the future.
