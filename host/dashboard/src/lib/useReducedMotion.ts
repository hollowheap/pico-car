import { useEffect, useState } from 'react';
import { AccessibilityInfo, Platform } from 'react-native';

export function useReducedMotion(): boolean {
  const [reduced, setReduced] = useState(true);
  useEffect(() => {
    if (Platform.OS === 'web') {
      const preference = window.matchMedia('(prefers-reduced-motion: reduce)');
      const update = () => setReduced(preference.matches);
      update();
      preference.addEventListener('change', update);
      return () => preference.removeEventListener('change', update);
    }
    let awaitingInitial = true;
    const subscription = AccessibilityInfo.addEventListener('reduceMotionChanged', (value) => {
      awaitingInitial = false;
      setReduced(value);
    });
    void AccessibilityInfo.isReduceMotionEnabled().then((value) => {
      if (awaitingInitial) setReduced(value);
    }, () => {});
    return () => {
      awaitingInitial = false;
      subscription.remove();
    };
  }, []);
  return reduced;
}
