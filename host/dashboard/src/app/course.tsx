import { Redirect, useLocalSearchParams } from 'expo-router';

export default function LegacyCourse() {
  const { event } = useLocalSearchParams<{ event?: string }>();
  return <Redirect href={{ pathname: '/', params: { section: 'course', event } }} />;
}
