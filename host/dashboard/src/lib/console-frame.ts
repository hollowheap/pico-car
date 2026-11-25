export function consoleFrame(width: number, height: number) {
  const largeDesktop = width >= 1500 && height >= 860;
  return { largeDesktop, inset: largeDesktop ? 14 : 10, maxWidth: largeDesktop ? 1760 : 1440 };
}
