import { createButton } from '@gluestack-ui/core/button/creator';
import { createContext, useContext, useState, type ComponentProps } from 'react';
import {
  ActivityIndicator,
  Platform,
  Pressable,
  StyleSheet,
  Text,
  View,
  type StyleProp,
  type ViewStyle,
} from 'react-native';

import { useTheme } from '../../lib/theme-provider';

const BaseButton = createButton({
  Root: Pressable,
  Text,
  Group: View,
  Spinner: ActivityIndicator,
  Icon: View,
});

type Variant = 'primary' | 'secondary';
type ButtonProps = ComponentProps<typeof BaseButton> & {
  variant?: Variant;
  compact?: boolean;
};

const VariantContext = createContext<Variant>('primary');
const DisabledContext = createContext(false);

function ThemedButton({
  variant = 'primary',
  compact = false,
  style,
  onFocus,
  onBlur,
  ...props
}: ButtonProps) {
  const theme = useTheme();
  const [focused, setFocused] = useState(false);
  const focusRing =
    Platform.OS === 'web'
      ? ({
          outlineStyle: 'solid',
          outlineWidth: 2,
          outlineColor: theme.colors.accent,
          outlineOffset: 2,
        } as ViewStyle)
      : {};
  const base: ViewStyle = {
    minHeight: 44,
    minWidth: 44,
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: compact ? theme.spacing.md : theme.spacing.xl,
    borderRadius: theme.radii.button,
    borderWidth: variant === 'secondary' ? 1 : 0,
    borderColor: theme.colors.border,
    backgroundColor: props.disabled
      ? theme.colors.elevated
      : variant === 'primary'
        ? theme.colors.accent
        : 'transparent',
  };

  return (
    <VariantContext.Provider value={variant}>
      <DisabledContext.Provider value={Boolean(props.disabled)}>
        <BaseButton
          {...props}
          onFocus={(event) => {
            setFocused(true);
            onFocus?.(event);
          }}
          onBlur={(event) => {
            setFocused(false);
            onBlur?.(event);
          }}
          style={[
            base,
            props.disabled ? styles.disabled : null,
            focused && focusRing,
            style as StyleProp<ViewStyle>,
          ]}
        />
      </DisabledContext.Provider>
    </VariantContext.Provider>
  );
}

function ThemedButtonText({
  style,
  ...props
}: ComponentProps<typeof BaseButton.Text>) {
  const theme = useTheme();
  const variant = useContext(VariantContext);
  const disabled = useContext(DisabledContext);
  const color = disabled
    ? theme.colors.muted
    : variant === 'primary'
      ? theme.colors.onAccent
      : theme.colors.text;

  return (
    <BaseButton.Text
      {...props}
      style={[theme.typography.body, { color, fontWeight: '600' }, style]}
    />
  );
}

const styles = StyleSheet.create({
  disabled: { opacity: 0.45 },
});

export const Button = Object.assign(ThemedButton, {
  Text: ThemedButtonText,
  Group: BaseButton.Group,
  Spinner: BaseButton.Spinner,
  Icon: BaseButton.Icon,
});
