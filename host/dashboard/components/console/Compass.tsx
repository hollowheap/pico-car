import Svg, {
  Circle,
  G,
  Line,
  Polygon,
  Text as SvgText,
} from 'react-native-svg';

import { useTheme } from '../../lib/theme-provider';

type Props = {
  headingDeg: number;
  available?: boolean;
};

export function Compass({ headingDeg, available = true }: Props) {
  const theme = useTheme();
  const heading = ((headingDeg % 360) + 360) % 360;

  return (
    <Svg
      accessibilityLabel={
        available
          ? `Heading ${heading.toFixed(0)} degrees`
          : 'Heading unavailable'
      }
      height={210}
      preserveAspectRatio="xMidYMid meet"
      viewBox="0 0 160 160"
      width="100%"
    >
      <Circle
        cx="80"
        cy="80"
        fill="none"
        r="74"
        stroke={theme.colors.border}
        strokeWidth="1"
      />
      <Circle
        cx="80"
        cy="80"
        fill="none"
        r="68"
        stroke={theme.colors.border}
        strokeDasharray="2 3"
        strokeWidth="1"
      />
      <Circle
        cx="80"
        cy="80"
        fill="none"
        r="52"
        stroke={theme.colors.border}
        strokeWidth="0.75"
      />

      <Line
        stroke={theme.colors.accent}
        strokeWidth="2"
        x1="80"
        x2="80"
        y1="8"
        y2="16"
      />
      <Line
        stroke={theme.colors.muted}
        strokeWidth="1.5"
        x1="152"
        x2="144"
        y1="80"
        y2="80"
      />
      <Line
        stroke={theme.colors.muted}
        strokeWidth="1.5"
        x1="80"
        x2="80"
        y1="152"
        y2="144"
      />
      <Line
        stroke={theme.colors.muted}
        strokeWidth="1.5"
        x1="8"
        x2="16"
        y1="80"
        y2="80"
      />

      <SvgText
        fill={theme.colors.accent}
        fontFamily="monospace"
        fontSize="10"
        fontWeight="bold"
        textAnchor="middle"
        x="80"
        y="27"
      >
        N
      </SvgText>
      <SvgText
        fill={theme.colors.muted}
        fontFamily="monospace"
        fontSize="9"
        textAnchor="middle"
        x="136"
        y="83"
      >
        E
      </SvgText>
      <SvgText
        fill={theme.colors.muted}
        fontFamily="monospace"
        fontSize="9"
        textAnchor="middle"
        x="80"
        y="139"
      >
        S
      </SvgText>
      <SvgText
        fill={theme.colors.muted}
        fontFamily="monospace"
        fontSize="9"
        textAnchor="middle"
        x="24"
        y="83"
      >
        W
      </SvgText>

      {available ? (
        <G transform={`rotate(${heading} 80 80)`}>
        <Polygon
          fill={theme.colors.accent}
          points="80,24 85,80 80,80"
          opacity={0.85}
        />
        <Polygon
          fill={theme.colors.accent}
          points="80,24 80,80 75,80"
        />
        <Polygon
          fill={theme.colors.muted}
          points="80,136 85,80 80,80"
          opacity={0.7}
        />
        <Polygon
          fill={theme.colors.muted}
          points="80,136 80,80 75,80"
        />
        <Line
          stroke={theme.colors.text}
          strokeWidth="0.75"
          x1="80"
          x2="80"
          y1="26"
          y2="134"
        />
        <Circle
          cx="80"
          cy="80"
          fill={theme.colors.card}
          r="7"
          stroke={theme.colors.accent}
          strokeWidth="1.5"
        />
        <Circle
          cx="80"
          cy="80"
          fill={theme.colors.accent}
          r="2.5"
        />
      </G>
      ) : null}
    </Svg>
  );
}
