import type { ComponentRef } from 'react';
import { AccessibilityInfo, Platform, type View } from 'react-native';

export function focusElement(
  node: ComponentRef<typeof View> | null | undefined,
  options?: { scrollBlock?: ScrollLogicalPosition },
): boolean {
  if (node === null || node === undefined) return false;
  if (Platform.OS === 'web') {
    const element = node as unknown as HTMLElement;
    if (options?.scrollBlock) {
      element.scrollIntoView({ block: options.scrollBlock, inline: 'nearest' });
    }
    element.focus({ preventScroll: true });
    return element === document.activeElement;
  }
  AccessibilityInfo.sendAccessibilityEvent(node, 'focus');
  return true;
}
