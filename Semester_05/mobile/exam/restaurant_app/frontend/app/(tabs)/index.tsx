import React, { useState, useEffect, useRef } from 'react';
import {
  View,
  StyleSheet,
  TextInput,
  TouchableOpacity,
  FlatList,
  Alert,
  ScrollView,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import LoadingSpinner from '@/components/LoadingSpinner';
import { createOrder, getReadyOrders, getOrderById } from '@/utils/api';
import { saveOrders, getLocalOrders, savePendingOrder, getPendingOrders, clearPendingOrders } from '@/utils/storage';
import { Order } from '@/types/order';
import { useWebSocket } from '@/hooks/useWebSocket';

export default function WaiterSection() {
  const [isOnline, setIsOnline] = useState(false);
  const [table, setTable] = useState('');
  const [details, setDetails] = useState('');
  const [time, setTime] = useState('');
  const [type, setType] = useState('');
  const [orders, setOrders] = useState<Order[]>([]);
  const [loading, setLoading] = useState(false);
  const [submitting, setSubmitting] = useState(false);
  const [loadingDetails, setLoadingDetails] = useState<number | null>(null);
  const [isOfflineMode, setIsOfflineMode] = useState(false);
  const wasOffline = useRef(false);
  const justSubmittedId = useRef<number | null>(null);

  useEffect(() => {
    const unsubscribe = NetInfo.addEventListener((state) => {
      const online = state.isConnected ?? false;
      setIsOnline(online);
    });
    NetInfo.fetch().then((state) => setIsOnline(state.isConnected ?? false));
    return () => unsubscribe();
  }, []);

  useEffect(() => {
    if (isOnline && wasOffline.current) {
      syncPendingOrders();
      loadReadyOrders();
    }
    wasOffline.current = !isOnline;
  }, [isOnline]);

  useEffect(() => {
    loadReadyOrders();
  }, []);

  const syncPendingOrders = async () => {
    const pending = await getPendingOrders();
    if (pending.length === 0) return;

    let synced = 0;
    for (const order of pending) {
      const response = await createOrder({
        table: order.table,
        details: order.details,
        time: order.time,
        type: order.type,
        status: order.status,
      });
      if (!response.error) {
        synced++;
      }
    }

    await clearPendingOrders();
    if (synced > 0) {
      Alert.alert('Synced', `${synced} pending order(s) synced to server`);
    }
  };

  useWebSocket({
    onMessage: (message: any) => {
      if (message.table && message.details && message.type) {
        if (message.id === justSubmittedId.current) {
          justSubmittedId.current = null;
          return;
        }
        Alert.alert(
          'New Order',
          `Table: ${message.table}\nDetails: ${message.details}\nType: ${message.type}`
        );
        loadReadyOrders();
      }
    },
  });

  const loadReadyOrders = async () => {
    setLoading(true);
    const response = await getReadyOrders();
    setLoading(false);

    if (response.error) {
      const cached = await getLocalOrders();
      setOrders(cached);
      setIsOfflineMode(true);
    } else if (response.data) {
      setOrders(response.data);
      setIsOfflineMode(false);
      await saveOrders(response.data);
    }
  };

  const handleSubmit = async () => {
    if (!table || !details || !time || !type) {
      Alert.alert('Error', 'Please fill all fields');
      return;
    }

    const timeNum = parseInt(time, 10);
    if (isNaN(timeNum) || timeNum <= 0) {
      Alert.alert('Error', 'Time must be a positive number (seconds)');
      return;
    }

    setSubmitting(true);
    const newOrder: Omit<Order, 'id'> = {
      table,
      details,
      time: timeNum,
      type,
      status: 'recorded',
    };

    const saveOffline = async () => {
      await savePendingOrder({ ...newOrder, id: Date.now() });
      setSubmitting(false);
      setTable('');
      setDetails('');
      setTime('');
      setType('');
      Alert.alert('Saved Offline', 'Order saved locally. Will sync when online.');
    };

    if (isOnline) {
      const response = await createOrder(newOrder);
      setSubmitting(false);
      if (response.error) {
        if (response.error.includes('Network') || response.error.includes('network')) {
          await saveOffline();
        } else {
          Alert.alert('Error', response.error);
        }
      } else if (response.data) {
        justSubmittedId.current = response.data.id;
        setTable('');
        setDetails('');
        setTime('');
        setType('');
        Alert.alert('Success', 'Order recorded');
        loadReadyOrders();
      }
    } else {
      await saveOffline();
    }
  };

  const handleOrderPress = async (order: Order) => {
    if (!isOnline) {
      Alert.alert('Offline', 'Cannot view details while offline');
      return;
    }

    setLoadingDetails(order.id);
    const response = await getOrderById(order.id);
    setLoadingDetails(null);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      const orderData = response.data;
      Alert.alert(
        'Order Details',
        `ID: ${orderData.id}\nTable: ${orderData.table}\nDetails: ${orderData.details}\nStatus: ${orderData.status}\nTime: ${orderData.time}\nType: ${orderData.type}`
      );
    }
  };

  const renderItem = ({ item }: { item: Order }) => {
    const isLoading = loadingDetails === item.id;
    return (
      <TouchableOpacity
        onPress={() => handleOrderPress(item)}
        disabled={!isOnline || isLoading}>
        <ThemedView style={styles.orderItem}>
          <ThemedText type="defaultSemiBold" style={styles.orderTable}>
            Table: {item.table}
          </ThemedText>
          <ThemedText style={styles.orderDetails}>{item.details}</ThemedText>
          <ThemedText style={styles.orderStatus}>Status: {item.status}</ThemedText>
          {isLoading && (
            <View style={styles.loadingOverlay}>
              <LoadingSpinner visible={true} />
            </View>
          )}
        </ThemedView>
      </TouchableOpacity>
    );
  };

  return (
    <View style={styles.container}>
      <ScrollView style={styles.formContainer}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>
          Record Order
        </ThemedText>
        <TextInput
          style={styles.input}
          placeholder="Table"
          value={table}
          onChangeText={setTable}
          autoCapitalize="none"
        />
        <TextInput
          style={styles.input}
          placeholder="Details"
          value={details}
          onChangeText={setDetails}
          autoCapitalize="none"
          multiline
        />
        <TextInput
          style={styles.input}
          placeholder="Time"
          value={time}
          onChangeText={setTime}
          autoCapitalize="none"
          keyboardType="numeric"
        />
        <TextInput
          style={styles.input}
          placeholder="Type"
          value={type}
          onChangeText={setType}
          autoCapitalize="none"
        />
        <TouchableOpacity
          style={[styles.submitButton, submitting && styles.submitButtonDisabled]}
          onPress={handleSubmit}
          disabled={submitting}>
          {submitting ? (
            <ActivityIndicator size="small" color="#fff" />
          ) : (
            <ThemedText style={styles.submitButtonText}>Submit Order</ThemedText>
          )}
        </TouchableOpacity>
      </ScrollView>

      <View style={styles.ordersContainer}>
        <View style={styles.ordersHeader}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            Ready Orders
          </ThemedText>
          {isOfflineMode && (
            <View style={styles.offlineBanner}>
              <ThemedText style={styles.offlineText}>Showing cached data</ThemedText>
              <TouchableOpacity onPress={loadReadyOrders} style={styles.retryButton}>
                <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
              </TouchableOpacity>
            </View>
          )}
        </View>
        {loading ? (
          <LoadingSpinner visible={true} message="Loading orders..." />
        ) : orders.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>No ready orders</ThemedText>
        ) : (
          <FlatList
            data={orders}
            renderItem={renderItem}
            keyExtractor={(item) => item.id.toString()}
            contentContainerStyle={styles.listContent}
            ItemSeparatorComponent={() => <View style={styles.separator} />}
          />
        )}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  formContainer: {
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#E0E0E0',
    maxHeight: '50%',
  },
  sectionTitle: {
    marginBottom: 16,
  },
  input: {
    borderWidth: 1,
    borderColor: '#E0E0E0',
    borderRadius: 8,
    padding: 12,
    marginBottom: 12,
    fontSize: 16,
    backgroundColor: '#fff',
  },
  submitButton: {
    backgroundColor: '#007AFF',
    paddingVertical: 14,
    borderRadius: 8,
    alignItems: 'center',
    marginTop: 8,
  },
  submitButtonDisabled: {
    opacity: 0.6,
  },
  submitButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
  ordersContainer: {
    flex: 1,
  },
  ordersHeader: {
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#E0E0E0',
  },
  offlineBanner: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginTop: 8,
    padding: 8,
    backgroundColor: '#FFF3CD',
    borderRadius: 4,
  },
  offlineText: {
    fontSize: 14,
    color: '#856404',
  },
  retryButton: {
    paddingHorizontal: 12,
    paddingVertical: 4,
    backgroundColor: '#007AFF',
    borderRadius: 4,
  },
  retryButtonText: {
    color: '#fff',
    fontSize: 12,
    fontWeight: '600',
  },
  listContent: {
    padding: 16,
  },
  orderItem: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
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
  orderStatus: {
    fontSize: 14,
    color: '#666',
  },
  loadingOverlay: {
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: 'rgba(255, 255, 255, 0.8)',
    justifyContent: 'center',
    alignItems: 'center',
  },
  separator: {
    height: 12,
  },
  emptyMessage: {
    fontSize: 14,
    color: '#666',
    fontStyle: 'italic',
    textAlign: 'center',
    padding: 16,
    marginTop: 32,
  },
});
