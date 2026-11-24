// The longest controlled reverse is the 250 mm hug back-off. A much larger
// step between adjacent samples is insufficient evidence of actual travel.
const MAX_REVERSE_STEP_MM = 500;
const MAX_FORWARD_STEP_MM = 5000;

export function odometerDiscontinuous(previousMm: number, nextMm: number): boolean {
  const distance = nextMm - previousMm;
  return distance < -MAX_REVERSE_STEP_MM || distance > MAX_FORWARD_STEP_MM;
}
