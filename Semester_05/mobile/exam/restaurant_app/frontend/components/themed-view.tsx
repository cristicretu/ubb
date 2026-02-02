import { View, ViewProps } from 'react-native';

export function ThemedView({ style, ...rest }: ViewProps) {
  return <View style={[{ backgroundColor: '#fff' }, style]} {...rest} />;
}
