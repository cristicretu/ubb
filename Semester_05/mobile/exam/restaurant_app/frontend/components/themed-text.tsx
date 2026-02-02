import { Text, TextProps, StyleSheet } from 'react-native';

type Props = TextProps & { type?: 'default' | 'title' | 'defaultSemiBold' | 'subtitle' };

export function ThemedText({ style, type = 'default', ...rest }: Props) {
  return <Text style={[styles[type], style]} {...rest} />;
}

const styles = StyleSheet.create({
  default: { fontSize: 16, color: '#11181C' },
  defaultSemiBold: { fontSize: 16, fontWeight: '600', color: '#11181C' },
  title: { fontSize: 32, fontWeight: 'bold', color: '#11181C' },
  subtitle: { fontSize: 20, fontWeight: 'bold', color: '#11181C' },
});
