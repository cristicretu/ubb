import React from 'react';
import { View, ActivityIndicator, StyleSheet } from 'react-native';
import { ThemedText } from './themed-text';

interface LoadingSpinnerProps {
  visible: boolean;
  message?: string;
}

export default function LoadingSpinner({ visible, message }: LoadingSpinnerProps) {
  if (!visible) return null;

  return (
    <View style={styles.container}>
      <ActivityIndicator size="large" color="#007AFF" />
      {message && <ThemedText style={styles.message}>{message}</ThemedText>}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    padding: 20,
    alignItems: 'center',
    justifyContent: 'center',
  },
  message: {
    marginTop: 12,
    fontSize: 14,
    color: '#666',
  },
});
