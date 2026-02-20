# Bring-up Issues Log

- **Issue-001: MDDS30 ignores PWM/DIR but manual buttons work**  
  - Symptoms: Motors spin with driver test buttons, but stay idle when Arduino drives AN/IN.  
  - Likely causes: Missing common ground on the control header; driver not in PWM+DIR independent mode (DIP #3 ON only); Manual/Test button not released; DIR pins floating (not set to OUTPUT/defined state); wrong sketch/board/pin mapping or bad pin.  
  - Quick checks: Tie Arduino GND to MDDS30 logic GND at the AN/IN header; verify DIP switches and Manual/Test state; ensure DIR pins are OUTPUT and driven HIGH/LOW even when PWM=0; measure IN/AN at the driver header; run a minimal sweep sketch and LED sanity check on D5/D6/D9/D10.  
  - Resolution: Corrected the DIR pin mapping to match wiring (Right DIR → Arduino D10, Left DIR → Arduino D6), which was opposite of the assumed mapping in the sketch. Set DIP switches to `10110100` (PWM input mode with microcontroller) per MDDS30 guidance—mode was the root cause.  
  - Status: Closed after verifying proper PWM mode fixes the issue.

Add new issues here as they arise (Issue-002, Issue-003, …) with the same format.
