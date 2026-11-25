import { Activity, type ComponentRef, type ReactNode, type Ref } from 'react';
import { StyleSheet, View, type StyleProp, type ViewStyle } from 'react-native';

import { Panel, PanelEmbedded } from './Panel';
import type { IconName } from './PanelIcon';
import { SectionPicker } from './SectionPicker';

export function TabbedCard<T extends string>({ id, title, icon, style, fill, tabs, active, onChange, panelRef, bodies }: {
  id: string;
  title: string;
  icon: IconName;
  style?: StyleProp<ViewStyle>;
  fill?: boolean;
  tabs: Readonly<Record<T, string>>;
  active: T;
  onChange: (tab: T) => void;
  panelRef: (tab: T) => Ref<ComponentRef<typeof View>>;
  bodies: Record<T, ReactNode>;
}) {
  return (
    <Panel icon={icon} title={title} style={style} fill={fill} testID={`${id}-card`}
      right={<SectionPicker id={id} label={`${title} tabs`} options={tabs} value={active} onChange={onChange} />}>
      <PanelEmbedded.Provider value>
        {(Object.keys(tabs) as T[]).map((tab) => {
          const selected = tab === active;
          return (
            <Activity key={tab} mode={selected ? 'visible' : 'hidden'}>
              <View aria-labelledby={`${id}-tab-${tab}`} nativeID={`${id}-panel-${tab}`}
                ref={panelRef(tab)} role="tabpanel" style={fill && styles.pane} tabIndex={selected ? 0 : -1} testID={`console-${tab}`}>
                {bodies[tab]}
              </View>
            </Activity>
          );
        })}
      </PanelEmbedded.Provider>
    </Panel>
  );
}

const styles = StyleSheet.create({ pane: { flex: 1, minHeight: 0 } });
