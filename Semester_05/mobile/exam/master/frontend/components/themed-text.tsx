import { Text, TextProps, StyleSheet } from 'react-native';

interface ThemedTextProps extends TextProps {
  type?: 'default' | 'title' | 'subtitle' | 'defaultSemiBold';
}

export function ThemedText({ type = 'default', style, ...props }: ThemedTextProps) {
  return <Text style={[styles[type], style]} {...props} />;
}

const styles = StyleSheet.create({
  default: { fontSize: 16, color: '#000' },
  title: { fontSize: 28, fontWeight: 'bold', color: '#000' },
  subtitle: { fontSize: 20, fontWeight: '600', color: '#000' },
  defaultSemiBold: { fontSize: 16, fontWeight: '600', color: '#000' },
});
