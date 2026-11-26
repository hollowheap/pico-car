import { ENUMS } from './schema.generated';

export function barcodeSpanKind(values: Record<string, number>): number {
  const { barcodeElement } = ENUMS;
  const widthUm = values.width_um ?? 0;
  if (values.capture_loss || !values.width_valid || widthUm <= 0) return barcodeElement.unknown;
  // The pin level is the level after the edge. HIGH starts the next dark span.
  if (values.level_high === 1) return barcodeElement.space;
  if (values.level_high !== 0) return barcodeElement.unknown;
  return widthUm > 50_000 ? barcodeElement.block : barcodeElement.bar;
}
