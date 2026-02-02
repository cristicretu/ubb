import React, { useState } from 'react';
import {
  View,
  StyleSheet,
  TextInput,
  TouchableOpacity,
  Alert,
  ScrollView,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getOrderByTable } from '@/utils/api';
import { Order } from '@/types/order';
import { useWebSocket } from '@/hooks/useWebSocket';
import LoadingSpinner from '@/components/LoadingSpinner';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';

export default function ClientSection() {
  const [tableName, setTableName] = useState('');
  const [order, setOrder] = useState<Order | null>(null);
  const [loading, setLoading] = useState(false);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.id && message.table && message.details && message.status !== undefined) {
        if (tableName && message.table === tableName) {
          checkOrder();
        }
      }
    },
  });

  const checkOrder = async () => {
    if (!tableName.trim()) {
      Alert.alert('Error', 'Please enter a table name');
      return;
    }

    const netInfo = await NetInfo.fetch();
    if (!netInfo.isConnected) {
      Alert.alert('Offline', 'This operation requires an internet connection');
      return;
    }

    setLoading(true);
    const response = await getOrderByTable(tableName.trim());
    setLoading(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      setOrder(null);
      return;
    }

    if (response.data) {
      setOrder(response.data);
    } else {
      setOrder(null);
    }
  };

  return (
    <ThemedView style={styles.container}>
      <ScrollView style={styles.scrollView} contentContainerStyle={styles.scrollContent}>
        <View style={styles.section}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            Enter Table Name
          </ThemedText>
          <TextInput
            style={styles.input}
            placeholder="Table name"
            value={tableName}
            onChangeText={setTableName}
            autoCapitalize="none"
          />
          <TouchableOpacity
            style={[styles.button, loading && styles.buttonDisabled]}
            onPress={checkOrder}
            disabled={loading}>
            <ThemedText style={styles.buttonText}>Check Order</ThemedText>
          </TouchableOpacity>
        </View>

        {loading ? (
          <LoadingSpinner visible={true} message="Loading order..." />
        ) : order ? (
          <View style={styles.section}>
            <ThemedText type="subtitle" style={styles.sectionTitle}>
              Order Details
            </ThemedText>
            <ThemedView style={styles.orderContainer}>
              <View style={styles.orderRow}>
                <ThemedText type="defaultSemiBold" style={styles.orderLabel}>
                  ID:
                </ThemedText>
                <ThemedText style={styles.orderValue}>{order.id}</ThemedText>
              </View>
              <View style={styles.orderRow}>
                <ThemedText type="defaultSemiBold" style={styles.orderLabel}>
                  Table:
                </ThemedText>
                <ThemedText style={styles.orderValue}>{order.table}</ThemedText>
              </View>
              <View style={styles.orderRow}>
                <ThemedText type="defaultSemiBold" style={styles.orderLabel}>
                  Details:
                </ThemedText>
                <ThemedText style={styles.orderValue}>{order.details}</ThemedText>
              </View>
              <View style={styles.orderRow}>
                <ThemedText type="defaultSemiBold" style={styles.orderLabel}>
                  Status:
                </ThemedText>
                <ThemedText style={styles.orderValue}>{order.status}</ThemedText>
              </View>
              <View style={styles.orderRow}>
                <ThemedText type="defaultSemiBold" style={styles.orderLabel}>
                  Time:
                </ThemedText>
                <ThemedText style={styles.orderValue}>{order.time}</ThemedText>
              </View>
              <View style={styles.orderRow}>
                <ThemedText type="defaultSemiBold" style={styles.orderLabel}>
                  Type:
                </ThemedText>
                <ThemedText style={styles.orderValue}>{order.type}</ThemedText>
              </View>
            </ThemedView>
          </View>
        ) : tableName ? (
          <View style={styles.section}>
            <View style={styles.emptyContainer}>
              <ThemedText style={styles.emptyMessage}>
                No order found for this table
              </ThemedText>
            </View>
          </View>
        ) : null}
      </ScrollView>
    </ThemedView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  scrollView: {
    flex: 1,
  },
  scrollContent: {
    padding: 16,
  },
  section: {
    marginBottom: 24,
  },
  sectionTitle: {
    marginBottom: 12,
  },
  input: {
    borderWidth: 1,
    borderColor: '#E5E5E5',
    borderRadius: 8,
    padding: 12,
    fontSize: 16,
    backgroundColor: '#fff',
    marginBottom: 12,
    minHeight: 44,
  },
  button: {
    backgroundColor: '#007AFF',
    paddingVertical: 12,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
  },
  buttonDisabled: {
    opacity: 0.6,
  },
  buttonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
  orderContainer: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  orderRow: {
    flexDirection: 'row',
    marginBottom: 12,
    alignItems: 'flex-start',
  },
  orderLabel: {
    minWidth: 80,
    marginRight: 8,
  },
  orderValue: {
    flex: 1,
  },
  emptyContainer: {
    padding: 40,
    alignItems: 'center',
  },
  emptyMessage: {
    fontSize: 14,
    color: '#999',
  },
});
