# Fontus Acoustic Message Transmission Scheduler
The `FAMTxSched()` function **SHALL** read a queue of FAM messages tied to PCM sample counts corresponding to the time of transmission desired.
`FAMTxSched()` **SHALL** call sort on the queue of FAM messages to make sure the soonest scheduled transmission is first.
Then it shall call `generateAudio()` and store the result in a buffer along with the number of samples generated.
`FAMTxSched()` **SHALL** finish by setting 2 timers:

- `Timer1` **SHALL** be set to wait until `(PcmSampleCount == (DesiredPcmCount - framesize))` where the `DesiredPcmCount` is from the soonest FAM queue element. This timer **SHALL** wake the modem, set the DMA memcpy pointer to the generated buffer minus the number of samples offset from frame boundary,
- `Timer2` **SHALL** be set to wait for the generated samples after `Timer1` triggers to set the DMA pointer back to the DMA 0 buffer and put the modem back to sleep.

## Reasoning:
by scheduling transmissions this way the modem is able to calculate the audio to be transmitted in the background's singular task (the main function) and when it comes time to transmit there is no computation that needs to happen only a pointer redirect in an isr or timer event