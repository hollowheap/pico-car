import Svg, { Circle, G, Line, Path, Polygon } from 'react-native-svg';

import { rotateToCarUp, type CoursePoint } from '../../lib/course';
import { nearestBoundary, type ObstacleView } from '../../lib/obstacle-view';
import { SONAR_FORWARD_MM } from '../../lib/scan';
import { useTheme } from '../../lib/theme-provider';
import { SvgLabel } from './SvgLabel';
import { SurfaceStroke } from './SurfaceStroke';

const WIDTH = 256;
const HEIGHT = 202;
const LEFT = 12;
const RIGHT = WIDTH - 51;
const TOP = 14;
const BOTTOM = HEIGHT - 25;
const CAR_HALF_WIDTH_MM = 55;
const CAR_FRONT_MM = 90;
const CAR_REAR_MM = 60;

function gridLines(origin: number, step: number, min: number, max: number): number[] {
  const first = min + ((origin - min) % step + step) % step;
  return Array.from({ length: Math.floor((max - first) / step) + 1 }, (_, i) => first + i * step);
}

export function ScanPlot({ contact: rawContact, surfaces, hits = [], car, carLabel, heading }: ObstacleView) {
  const { colors } = useTheme();
  const rotate = carLabel === 'Car' && heading !== null;
  const xform = (p: CoursePoint) => (rotate ? rotateToCarUp(p, car, heading) : p);
  const viewCar = rotate ? { x: 0, y: 0 } : car;
  const viewSurfaces = rotate
    ? surfaces.map((face) => ({ ...face, points: face.points.map(xform) }))
    : surfaces;
  const viewHits = rotate ? hits.map(xform) : hits;
  const contact = rawContact && xform(rawContact);
  const outlines = viewSurfaces.map(face => face.points);
  const points = [viewCar, ...outlines.flat(), ...viewHits, ...(contact ? [contact] : [])];
  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const scale = Math.min((RIGHT - LEFT) / Math.max(180, maxX - minX), (BOTTOM - TOP) / Math.max(125, maxY - minY));
  const project = (p: CoursePoint) => ({
    x: (LEFT + RIGHT) / 2 + (p.x - (minX + maxX) / 2) * scale,
    y: (TOP + BOTTOM) / 2 - (p.y - (minY + maxY) / 2) * scale,
  });

  const axle = project(viewCar);
  const closest = carLabel === 'Car' ? nearestBoundary(viewCar, outlines) : undefined;
  const target = closest && project(closest);
  const distance = closest && Math.round(Math.hypot(viewCar.x - closest.x, viewCar.y - closest.y));
  const contactPoint = contact && project(contact);
  const decade = 10 ** Math.floor(Math.log10(35 / scale));
  const scaleMm = [1, 2, 5, 10].find((unit) => unit * decade * scale >= 35)! * decade;
  const scaleWidth = scaleMm * scale;
  const origin = project({ x: 0, y: 0 });
  const viewHeadingDeg = rotate ? 0 : heading;
  const headingRad = viewHeadingDeg === null ? null : viewHeadingDeg * Math.PI / 180;
  const labelLeft = !!target && target.x > axle.x + 2 && axle.x > 60;
  const bounds = viewSurfaces.reduce((n, face) => n + Number(face.boundedStart) + Number(face.boundedEnd), 0);
  const description = contact ? 'Front contact' : `${viewSurfaces.length} measured faces, ${bounds} bounded endpoints`;

  const carOutline = headingRad === null ? null : (() => {
    const cos = Math.cos(headingRad);
    const sin = Math.sin(headingRad);
    const localToWorld = (lx: number, ly: number) => project({
      x: viewCar.x + cos * lx + sin * ly,
      y: viewCar.y - sin * lx + cos * ly,
    });
    const corners = [
      localToWorld(-CAR_HALF_WIDTH_MM, CAR_FRONT_MM),
      localToWorld(CAR_HALF_WIDTH_MM, CAR_FRONT_MM),
      localToWorld(CAR_HALF_WIDTH_MM, -CAR_REAR_MM),
      localToWorld(-CAR_HALF_WIDTH_MM, -CAR_REAR_MM),
    ];
    const sonar = localToWorld(0, SONAR_FORWARD_MM);
    return { polygon: corners.map((c) => `${c.x.toFixed(1)},${c.y.toFixed(1)}`).join(' '), sonar };
  })();

  return (
    <Svg accessibilityLabel={`${description}. ${carLabel} shown as a dot.${distance === undefined ? '' : ` Axle to edge about ${distance} mm.`}`}
      accessibilityRole="image" style={{ alignSelf: 'center', maxWidth: WIDTH }} height={HEIGHT} viewBox={`0 0 ${WIDTH} ${HEIGHT}`} width="100%" testID="obstacle-edges">
      <G stroke={colors.border} strokeWidth={0.7}>
        {gridLines(origin.x, scaleWidth, 6, WIDTH - 6).map(x => <Line key={x} x1={x} x2={x} y1={4} y2={BOTTOM + 6} />)}
        {gridLines(origin.y, scaleWidth, 4, BOTTOM + 6).map(y => <Line key={y} y1={y} y2={y} x1={6} x2={WIDTH - 6} />)}
      </G>
      {carOutline ? (
        <G>
          <Polygon points={carOutline.polygon} fill={colors.accent} fillOpacity={0.08} stroke={colors.muted} strokeWidth={1} />
          <Circle cx={carOutline.sonar.x} cy={carOutline.sonar.y} r={2.2} fill={colors.accent} opacity={0.8} />
        </G>
      ) : null}
      {viewHits.map((hit, i) => {
        const pt = project(hit);
        return <Circle key={`hit-${i}`} cx={pt.x} cy={pt.y} r={2.3} fill={colors.accent} opacity={0.7} />;
      })}
      {contactPoint ? <Path testID="scan-face-contact" d={`M${contactPoint.x - 3} ${contactPoint.y - 3} l6 6 m0 -6 l-6 6`}
        stroke={colors.text} strokeWidth={2} /> : null}
      {viewSurfaces.map((face, i) => <SurfaceStroke key={i} surface={face} project={project} color={colors.text}
        pathId={`scan-surface-${face.firstSlot}`} startId={`scan-edge-${face.firstSlot}`}
        endId={`scan-edge-${face.firstSlot + face.points.length - 1}`} />)}
      {target ? <Line testID="scan-axle-projection" x1={axle.x} y1={axle.y} x2={target.x} y2={target.y}
        stroke={colors.muted} strokeWidth={1.4} strokeDasharray="4 3" /> : null}
      {headingRad !== null ? <Line x1={axle.x} y1={axle.y} x2={axle.x + 11 * Math.sin(headingRad)}
        y2={axle.y - 11 * Math.cos(headingRad)} stroke={colors.accent} strokeWidth={2.5} strokeLinecap="round" /> : null}
      <Circle testID="scan-car-reference" cx={axle.x} cy={axle.y} r={4.8}
        fill={carLabel === 'Car' || carLabel === 'At scan' ? colors.accent : colors.elevated} stroke={colors.text} strokeWidth={1.5} />
      <SvgLabel x={axle.x + (labelLeft ? -9 : 9)} y={axle.y + 4} textAnchor={labelLeft ? 'end' : 'start'}
        fontSize={12} fill={colors.text}>{carLabel}</SvgLabel>
      <G testID="scan-distance-scale">
        <Path d={`M${LEFT} ${HEIGHT - 13} v3 h${scaleWidth} v-3`} fill="none" stroke={colors.muted} />
        <SvgLabel x={LEFT + scaleWidth + 6} y={HEIGHT - 7} fontSize={12} fill={colors.muted}>{scaleMm} mm</SvgLabel>
      </G>
      {distance !== undefined ? <>
        <SvgLabel x={WIDTH - 62} y={HEIGHT - 7} textAnchor="end" fontSize={12} fill={colors.muted}>Axle to edge</SvgLabel>
        <SvgLabel x={WIDTH - 4} y={HEIGHT - 7} textAnchor="end" fontSize={12} fill={colors.text}>{distance} mm</SvgLabel>
      </> : null}
    </Svg>
  );
}
