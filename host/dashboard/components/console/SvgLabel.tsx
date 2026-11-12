import { Text as SvgText, type TextProps } from 'react-native-svg';

/** The face of the dashboard, so SVG text matches the page and not the browser default (serif). */
export const SVG_FONT = 'system-ui, -apple-system, "Segoe UI", sans-serif';

type Props = TextProps & { fill: string };

/**
 * SVG text with the page font. `color` repeats the fill so tools that read the
 * CSS colour of the element (contrast checks) see the real colour.
 */
export function SvgLabel({ fill, ...props }: Props) {
  return <SvgText color={fill} fill={fill} fontFamily={SVG_FONT} {...props} />;
}
