import Svg, { Line, Polyline } from 'react-native-svg';

type Props = {
  values: number[];
  color: string;
  label: string;
  height?: number;
  /** Fixed value range. Without it the line fills the height, which exaggerates noise. */
  range?: [number, number];
};

const WIDTH = 100;
const VIEW_HEIGHT = 40;

export function Sparkline({ values, color, label, height = 64, range }: Props) {
  const points =
    values.length < 2
      ? `0,${VIEW_HEIGHT / 2} ${WIDTH},${VIEW_HEIGHT / 2}`
      : (() => {
          const min = range?.[0] ?? Math.min(...values);
          const max = range?.[1] ?? Math.max(...values);
          const span = max - min || 1;
          return values
            .map((value, index) => {
              const x = (index / (values.length - 1)) * WIDTH;
              const y =
                VIEW_HEIGHT - 2 - ((value - min) / span) * (VIEW_HEIGHT - 4);
              return `${x.toFixed(2)},${y.toFixed(2)}`;
            })
            .join(' ');
        })();

  return (
    <Svg
      accessibilityLabel={label}
      accessibilityRole="image"
      height={height}
      preserveAspectRatio="none"
      viewBox={`0 0 ${WIDTH} ${VIEW_HEIGHT}`}
      width="100%"
    >
      <Line
        stroke={color}
        strokeDasharray="2 3"
        strokeWidth="0.5"
        x1="0"
        x2={WIDTH}
        y1={VIEW_HEIGHT / 2}
        y2={VIEW_HEIGHT / 2}
        opacity={0.35}
      />
      <Polyline
        fill="none"
        points={points}
        stroke={color}
        strokeLinecap="round"
        strokeLinejoin="round"
        strokeWidth="1.5"
      />
    </Svg>
  );
}
