import Svg, { Circle, Path } from 'react-native-svg';

type Props = {
  color: string;
  size?: number;
};

export function CarIcon({ color, size = 24 }: Props) {
  return (
    <Svg
      height={size}
      viewBox="0 0 24 24"
      width={size}
    >
      <Path
        d="M5 15.5V11l1.8-4.2A2 2 0 0 1 8.6 5.5h6.8a2 2 0 0 1 1.8 1.3L19 11v4.5"
        fill="none"
        stroke={color}
        strokeLinecap="round"
        strokeLinejoin="round"
        strokeWidth={1.8}
      />
      <Path
        d="M5 11h14"
        fill="none"
        stroke={color}
        strokeLinecap="round"
        strokeWidth={1.8}
      />
      <Circle
        cx="8"
        cy="16.5"
        fill="none"
        r="1.6"
        stroke={color}
        strokeWidth={1.8}
      />
      <Circle
        cx="16"
        cy="16.5"
        fill="none"
        r="1.6"
        stroke={color}
        strokeWidth={1.8}
      />
    </Svg>
  );
}
