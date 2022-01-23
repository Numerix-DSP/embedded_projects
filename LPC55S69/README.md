# LPC55S69
NXP LPC55S69 Examples

## LPC55S69_int_playrec.c

The standard audio I/O example for the LPC55S69-EVK implements a very simple piece of code
to read in an array of audio data from the stereo codec and then write it back. It does not
show how to access that data or process it using background DSP functions.

This example uses interrupts, ping-pong buffers and background tasks to apply DSP functions
to the real-time audio datastream.


