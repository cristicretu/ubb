import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  FlatList,
  TouchableOpacity,
  Alert,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getRecordedOrders, updateStatus } from '@/utils/api';
import { Order } from '@/types/order';
import { useWebSocket } from '@/hooks/useWebSocket';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';

export default function KitchenSection() {
  const [orders, setOrders] = useState<Order[]>([]);
  const [loading, setLoading] = useState(false);
  const [isOnline, setIsOnline] = useState(false);
  const [updatingIds, setUpdatingIds] = useState<Set<number>>(new Set());

  useEffect(() => {
    const unsubscribe = NetInfo.addEventListener((state) => {
      const online = state.isConnected ?? false;
      setIsOnline(online);
      log(`Network: ${online ? 'Online' : 'Offline'}`, 'info');
    });
    NetInfo.fetch().then((state) => setIsOnline(state.isConnected ?? false));
    return () => unsubscribe();
  }, []);

  useEffect(() => {
    if (isOnline) {
      loadOrders();
    }
  }, [isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.table && message.details && message.time !== undefined && message.type) {
        if (isOnline) {
          loadOrders();
        }
      }
    },
  });

  const loadOrders = async () => {
    if (!isOnline) {
      return;
    }

    setLoading(true);
    const response = await getRecordedOrders();
    setLoading(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      setOrders(response.data);
    }
  };

  const handleStatusUpdate = async (orderId: number, status: string) => {
    if (!isOnline) {
      Alert.alert('Offline', 'This operation requires an internet connection');
      return;
    }

    setUpdatingIds((prev) => new Set(prev).add(orderId));
    const response = await updateStatus(orderId, status);
    setUpdatingIds((prev) => {
      const next = new Set(prev);
      next.delete(orderId);
      return next;
    });

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    await loadOrders();
  };

  const renderOrderItem = ({ item }: { item: Order }) => {
    const isUpdating = updatingIds.has(item.id);

    return (
      <ThemedView style={styles.orderContainer}>
        <View style={styles.orderInfo}>
          <ThemedText type="defaultSemiBold" style={styles.orderTable}>
            Table: {item.table}
          </ThemedText>
          <ThemedText style={styles.orderDetails}>{item.details}</ThemedText>
          <View style={styles.orderMeta}>
            <ThemedText style={styles.orderMetaText}>Time: {item.time}s</ThemedText>
            <ThemedText style={styles.orderMetaText}>Type: {item.type}</ThemedText>
          </View>
          <ThemedText style={styles.orderStatus}>Status: {item.status}</ThemedText>
        </View>
        <View style={styles.buttonsContainer}>
          <TouchableOpacity
            style={[styles.statusButton, styles.readyButton, (!isOnline || isUpdating) && styles.buttonDisabled]}
            onPress={() => handleStatusUpdate(item.id, 'ready')}
            disabled={!isOnline || isUpdating}>
            {isUpdating ? (
              <ActivityIndicator size="small" color="#fff" />
            ) : (
              <ThemedText style={styles.buttonText}>Ready</ThemedText>
            )}
          </TouchableOpacity>
          <TouchableOpacity
            style={[styles.statusButton, styles.preparingButton, (!isOnline || isUpdating) && styles.buttonDisabled]}
            onPress={() => handleStatusUpdate(item.id, 'preparing')}
            disabled={!isOnline || isUpdating}>
            {isUpdating ? (
              <ActivityIndicator size="small" color="#fff" />
            ) : (
              <ThemedText style={styles.buttonText}>Preparing</ThemedText>
            )}
          </TouchableOpacity>
          <TouchableOpacity
            style={[styles.statusButton, styles.canceledButton, (!isOnline || isUpdating) && styles.buttonDisabled]}
            onPress={() => handleStatusUpdate(item.id, 'canceled')}
            disabled={!isOnline || isUpdating}>
            {isUpdating ? (
              <ActivityIndicator size="small" color="#fff" />
            ) : (
              <ThemedText style={styles.buttonText}>Canceled</ThemedText>
            )}
          </TouchableOpacity>
        </View>
      </ThemedView>
    );
  };

  return (
    <ThemedView style={styles.container}>
      {!isOnline && (
        <View style={styles.offlineBanner}>
          <ThemedText style={styles.offlineText}>
            You are offline. This section requires an internet connection.
          </ThemedText>
        </View>
      )}
      {loading ? (
        <LoadingSpinner visible={true} message="Loading orders..." />
      ) : orders.length === 0 ? (
        <View style={styles.emptyContainer}>
          <ThemedText style={styles.emptyMessage}>
            {isOnline ? 'No orders found' : 'Connect to the internet to view orders'}
          </ThemedText>
        </View>
      ) : (
        <FlatList
          data={orders}
          renderItem={renderOrderItem}
          keyExtractor={(item) => item.id.toString()}
          contentContainerStyle={styles.listContent}
          ItemSeparatorComponent={() => <View style={styles.separator} />}
        />
      )}
    </ThemedView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  offlineBanner: {
    padding: 12,
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
    margin: 16,
  },
  offlineText: {
    fontSize: 14,
    color: '#856404',
    textAlign: 'center',
  },
  listContent: {
    padding: 16,
  },
  orderContainer: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  orderInfo: {
    marginBottom: 12,
  },
  orderTable: {
    fontSize: 18,
    marginBottom: 8,
  },
  orderDetails: {
    fontSize: 14,
    color: '#666',
    marginBottom: 8,
  },
  orderMeta: {
    flexDirection: 'row',
    gap: 16,
    marginBottom: 8,
  },
  orderMetaText: {
    fontSize: 12,
    color: '#999',
  },
  orderStatus: {
    fontSize: 14,
    fontWeight: '600',
    color: '#007AFF',
  },
  buttonsContainer: {
    flexDirection: 'row',
    gap: 8,
  },
  statusButton: {
    flex: 1,
    paddingVertical: 10,
    paddingHorizontal: 12,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
    minHeight: 40,
  },
  readyButton: {
    backgroundColor: '#34C759',
  },
  preparingButton: {
    backgroundColor: '#FF9500',
  },
  canceledButton: {
    backgroundColor: '#FF3B30',
  },
  buttonDisabled: {
    opacity: 0.5,
  },
  buttonText: {
    color: '#fff',
    fontSize: 12,
    fontWeight: '600',
  },
  separator: {
    height: 12,
  },
  emptyContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 40,
  },
  emptyMessage: {
    fontSize: 14,
    color: '#999',
    textAlign: 'center',
  },
});
