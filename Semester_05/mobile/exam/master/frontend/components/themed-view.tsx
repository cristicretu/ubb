import { View, ViewProps } from 'react-native';

export function ThemedView({ style, ...props }: ViewProps) {
  return <View style={[{ backgroundColor: '#fff' }, style]} {...props} />;
}
