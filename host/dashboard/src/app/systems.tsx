import { Redirect, useLocalSearchParams } from 'expo-router';

export default function LegacySystems() {
  const { section } = useLocalSearchParams<{ section?: string }>();
  return <Redirect href={section === 'faults' || section === 'drive' ? { pathname: '/', params: { section } } : '/'} />;
}
