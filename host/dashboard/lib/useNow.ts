import { useEffect, useState } from 'react';

export function useNow(intervalMs = 250): number {
  const [now, setNow] = useState(() => Date.now());

  useEffect(() => {
    const handle = setInterval(() => setNow(Date.now()), intervalMs);
    return () => clearInterval(handle);
  }, [intervalMs]);

  return now;
}
