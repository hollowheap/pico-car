import Svg, { Circle, Line, Path, Rect } from 'react-native-svg';

import { useTheme } from '../../lib/theme-provider';

export type IconName = 'motion' | 'attitude' | 'dynamics' | 'line' | 'faults' | 'events' | 'link' | 'tasks' | 'run' | 'trace' | 'obstacle' | 'controls';

/** A 20 px line icon in the muted colour, beside a panel title. */
export function PanelIcon({ name }: { name: IconName }) {
  const theme = useTheme();
  const c = theme.colors.muted;
  const common = { fill: 'none', stroke: c, strokeWidth: 1.6, strokeLinecap: 'round' as const, strokeLinejoin: 'round' as const };

  return (
    <Svg aria-hidden height={20} viewBox="0 0 20 20" width={20}>
      {name === 'motion' ? (
        <>
          <Circle cx="5" cy="14" r="2.5" {...common} />
          <Circle cx="15" cy="14" r="2.5" {...common} />
          <Path d="M3 10h14M6 10l2-4h4l2 4" {...common} />
        </>
      ) : null}
      {name === 'attitude' ? (
        <>
          <Circle cx="10" cy="10" r="7" {...common} />
          <Path d="M10 4l2 6-2 6-2-6z" {...common} />
        </>
      ) : null}
      {name === 'dynamics' ? (
        <>
          <Circle cx="10" cy="10" r="7" {...common} />
          <Line x1="10" x2="10" y1="3" y2="17" {...common} />
          <Line x1="3" x2="17" y1="10" y2="10" {...common} />
          <Circle cx="13" cy="7" r="1.6" fill={c} />
        </>
      ) : null}
      {name === 'line' ? (
        <>
          <Path d="M3 17c4-2 4-12 14-14" {...common} />
          <Circle cx="7" cy="6" r="1.5" {...common} />
          <Circle cx="13" cy="12" r="1.5" {...common} />
        </>
      ) : null}
      {name === 'faults' ? (
        <>
          <Path d="M10 3l8 14H2z" {...common} />
          <Line x1="10" x2="10" y1="8" y2="12" {...common} />
          <Line x1="10" x2="10" y1="14.5" y2="14.6" {...common} />
        </>
      ) : null}
      {name === 'events' ? (
        <>
          <Line x1="6" x2="17" y1="5" y2="5" {...common} />
          <Line x1="6" x2="17" y1="10" y2="10" {...common} />
          <Line x1="6" x2="17" y1="15" y2="15" {...common} />
          <Circle cx="3" cy="5" r="0.8" {...common} />
          <Circle cx="3" cy="10" r="0.8" {...common} />
          <Circle cx="3" cy="15" r="0.8" {...common} />
        </>
      ) : null}
      {name === 'link' ? (
        <>
          <Path d="M3 8a10 10 0 0114 0M6 11a6 6 0 018 0" {...common} />
          <Circle cx="10" cy="15" r="1.2" {...common} />
        </>
      ) : null}
      {name === 'tasks' ? (
        <>
          <Rect height="5" width="5" x="3" y="3" {...common} />
          <Rect height="5" width="5" x="12" y="3" {...common} />
          <Rect height="5" width="5" x="3" y="12" {...common} />
          <Rect height="5" width="5" x="12" y="12" {...common} />
        </>
      ) : null}
      {name === 'run' ? (
        <Path d="M3 17V9M8 17V5M13 17v-6M18 17V3" {...common} />
      ) : null}
      {name === 'trace' ? (
        <Path d="M3 16c3 0 3-6 7-6s4-6 7-6" {...common} />
      ) : null}
      {name === 'obstacle' ? (
        <>
          <Path d="M10 16L4 7.5a9.5 9.5 0 0112 0L10 16z" {...common} />
          <Path d="M7.2 11.5a4.5 4.5 0 015.6 0" {...common} />
        </>
      ) : null}
      {name === 'controls' ? (
        <>
          <Line x1="3" x2="5.5" y1="6" y2="6" {...common} />
          <Line x1="9.5" x2="17" y1="6" y2="6" {...common} />
          <Circle cx="7.5" cy="6" r="1.8" {...common} />
          <Line x1="3" x2="11" y1="14" y2="14" {...common} />
          <Line x1="15" x2="17" y1="14" y2="14" {...common} />
          <Circle cx="13" cy="14" r="1.8" {...common} />
        </>
      ) : null}
    </Svg>
  );
}
