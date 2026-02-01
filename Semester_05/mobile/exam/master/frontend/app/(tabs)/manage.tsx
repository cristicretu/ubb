import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  FlatList,
  Alert,
  TouchableOpacity,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getAvailableItems, deleteItem, performAction } from '@/utils/api';
import { getOwnerName } from '@/utils/storage';
import { Item } from '@/types/item';
import { useWebSocket } from '@/hooks/useWebSocket';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';

export default function ManageSection() {
  const [items, setItems] = useState<Item[]>([]);
  const [loading, setLoading] = useState(false);
  const [isOnline, setIsOnline] = useState(false);
  const [deletingIds, setDeletingIds] = useState<Set<number>>(new Set());
  const [takingIds, setTakingIds] = useState<Set<number>>(new Set());

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
      loadItems();
    }
  }, [isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.id && message.name && message.value1 !== undefined) {
        Alert.alert(
          'New Item Added',
          `New Item: ${message.name}, Value1: ${message.value1}, Value2: ${message.value2}`
        );
        loadItems();
      }
    },
  });

  const loadItems = async () => {
    if (!isOnline) {
      return;
    }

    setLoading(true);
    const response = await getAvailableItems();
    setLoading(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      setItems(response.data);
    }
  };

  const handleDelete = async (item: Item) => {
    if (!isOnline) {
      Alert.alert('Offline', 'This operation requires an internet connection');
      return;
    }

    Alert.alert(
      'Delete Item',
      `Are you sure you want to delete "${item.name}"?`,
      [
        { text: 'Cancel', style: 'cancel' },
        {
          text: 'Delete',
          style: 'destructive',
          onPress: async () => {
            setDeletingIds((prev) => new Set(prev).add(item.id));
            const response = await deleteItem(item.id);
            setDeletingIds((prev) => {
              const next = new Set(prev);
              next.delete(item.id);
              return next;
            });

            if (response.error) {
              Alert.alert('Error', response.error);
              return;
            }

            Alert.alert('Success', 'Item deleted successfully');
            loadItems();
          },
        },
      ]
    );
  };

  const handleTake = async (item: Item) => {
    if (!isOnline) {
      Alert.alert('Offline', 'This operation requires an internet connection');
      return;
    }

    const owner = await getOwnerName();
    if (!owner) {
      Alert.alert('Error', 'Please save your name in My Section first');
      return;
    }

    setTakingIds((prev) => new Set(prev).add(item.id));
    const response = await performAction(item.id, 'taken', owner);
    setTakingIds((prev) => {
      const next = new Set(prev);
      next.delete(item.id);
      return next;
    });

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    Alert.alert('Success', 'Item taken successfully');
    loadItems();
  };

  const renderItem = ({ item }: { item: Item }) => {
    const isDeleting = deletingIds.has(item.id);
    const isTaking = takingIds.has(item.id);

    return (
      <ThemedView style={styles.itemContainer}>
        <View style={styles.itemInfo}>
          <ThemedText type="defaultSemiBold" style={styles.itemName}>
            {item.name}
          </ThemedText>
          <ThemedText style={styles.itemValues}>
            Value1: {item.value1} | Value2: {item.value2}
          </ThemedText>
        </View>
        <View style={styles.itemActions}>
          <TouchableOpacity
            style={[styles.deleteButton, (isDeleting || isTaking) && styles.buttonDisabled]}
            onPress={() => handleDelete(item)}
            disabled={isDeleting || isTaking}>
            {isDeleting ? (
              <ActivityIndicator size="small" color="#fff" />
            ) : (
              <ThemedText style={styles.deleteButtonText}>DELETE</ThemedText>
            )}
          </TouchableOpacity>
          <TouchableOpacity
            style={[styles.takeButton, (isDeleting || isTaking) && styles.buttonDisabled]}
            onPress={() => handleTake(item)}
            disabled={isDeleting || isTaking}>
            {isTaking ? (
              <ActivityIndicator size="small" color="#fff" />
            ) : (
              <ThemedText style={styles.takeButtonText}>Take</ThemedText>
            )}
          </TouchableOpacity>
        </View>
      </ThemedView>
    );
  };

  return (
    <ThemedView style={styles.container}>
      {!isOnline && (
        <View style={styles.offlineContainer}>
          <ThemedText style={styles.offlineMessage}>
            Offline - This section requires an internet connection
          </ThemedText>
        </View>
      )}
      {loading ? (
        <LoadingSpinner visible={true} message="Loading items..." />
      ) : (
        <>
          {items.length === 0 ? (
            <View style={styles.emptyContainer}>
              <ThemedText style={styles.emptyMessage}>No available items found</ThemedText>
            </View>
          ) : (
            <FlatList
              data={items}
              renderItem={renderItem}
              keyExtractor={(item) => item.id.toString()}
              contentContainerStyle={styles.listContent}
              ItemSeparatorComponent={() => <View style={styles.separator} />}
            />
          )}
        </>
      )}
    </ThemedView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  offlineContainer: {
    padding: 16,
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
    margin: 16,
    alignItems: 'center',
  },
  offlineMessage: {
    fontSize: 14,
    color: '#856404',
    textAlign: 'center',
  },
  listContent: {
    padding: 16,
  },
  itemContainer: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  itemInfo: {
    marginBottom: 12,
  },
  itemName: {
    fontSize: 16,
    marginBottom: 4,
  },
  itemValues: {
    fontSize: 14,
    color: '#666',
  },
  itemActions: {
    flexDirection: 'row',
    gap: 12,
  },
  deleteButton: {
    flex: 1,
    backgroundColor: '#FF3B30',
    paddingVertical: 10,
    paddingHorizontal: 16,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
  },
  takeButton: {
    flex: 1,
    backgroundColor: '#34C759',
    paddingVertical: 10,
    paddingHorizontal: 16,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
  },
  buttonDisabled: {
    opacity: 0.6,
  },
  deleteButtonText: {
    color: '#fff',
    fontSize: 14,
    fontWeight: '600',
  },
  takeButtonText: {
    color: '#fff',
    fontSize: 14,
    fontWeight: '600',
  },
  separator: {
    height: 8,
  },
  emptyContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 40,
  },
  emptyMessage: {
    fontSize: 16,
    color: '#999',
    textAlign: 'center',
  },
});
