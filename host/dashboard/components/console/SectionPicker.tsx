import { useRef, type ComponentProps, type ComponentRef } from 'react';
import { Platform, StyleSheet, View } from 'react-native';

import { useTheme } from '../../lib/theme-provider';
import { Button } from '../ui/button';

export function SectionPicker<T extends string>({ id, label, options, value, onChange }: {
  id: string;
  label: string;
  options: Readonly<Record<T, string>>;
  value: T;
  onChange: (section: T) => void;
}) {
  const theme = useTheme();
  const refs = useRef<Partial<Record<T, ComponentRef<typeof View> | null>>>({});
  const sections = Object.keys(options) as T[];

  return (
    <View accessibilityLabel={label} role="tablist" style={styles.row}>
      {sections.map((section, index) => {
        const selected = value === section;
        const keyboard = Platform.OS === 'web' ? {
          role: 'tab' as const,
          'aria-selected': selected,
          tabIndex: selected ? 0 as const : -1 as const,
          onKeyDown: (event: Parameters<NonNullable<ComponentProps<typeof Button>['onKeyDown']>>[0]) => {
            const { key } = event as typeof event & { key?: string };
            const target = key === ' ' || key === 'Enter' ? section
              : key === 'Home' ? sections[0]
              : key === 'End' ? sections.at(-1)
                : key === 'ArrowRight' ? sections[(index + 1) % sections.length]
                  : key === 'ArrowLeft' ? sections[(index + sections.length - 1) % sections.length] : undefined;
            if (target === undefined) return;
            event.preventDefault();
            onChange(target);
            refs.current[target]?.focus();
          },
          'aria-controls': `${id}-panel-${section}`,
        } : {};
        return (
          <Button
            {...keyboard}
            accessibilityRole="tab"
            accessibilityState={{ selected }}
            compact
            key={section}
            nativeID={`${id}-tab-${section}`}
            onPress={() => onChange(section)}
            ref={(node) => { refs.current[section] = node; }}
            style={styles.tab}
            variant={selected ? 'primary' : 'secondary'}
          >
            <Button.Text style={theme.typography.label}>{options[section]}</Button.Text>
          </Button>
        );
      })}
    </View>
  );
}

const styles = StyleSheet.create({
  row: { flexDirection: 'row', flexWrap: 'wrap', gap: 4 },
  tab: { paddingHorizontal: 9 },
});
