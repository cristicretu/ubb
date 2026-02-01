import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  TextInput,
  TouchableOpacity,
  Alert,
  FlatList,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import LoadingSpinner from '@/components/LoadingSpinner';
import { createItem, getItemsByOwner } from '@/utils/api';
import { saveItems, getLocalItems, savePendingItem, saveOwnerName, getOwnerName } from '@/utils/storage';
import { Item } from '@/types/item';
import { useWebSocket } from '@/hooks/useWebSocket';
import { log } from '@/utils/logger';

export default function MySection() {
  const [isOnline, setIsOnline] = useState(false);
  const [ownerName, setOwnerName] = useState('');
  const [savedOwner, setSavedOwner] = useState<string | null>(null);
  const [items, setItems] = useState<Item[]>([]);
  const [loading, setLoading] = useState(false);
  const [submitting, setSubmitting] = useState(false);
  
  const [itemName, setItemName] = useState('');
  const [itemStatus, setItemStatus] = useState('');
  const [itemValue1, setItemValue1] = useState('');

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
    loadSavedOwner();
  }, []);

  useEffect(() => {
    if (savedOwner) {
      loadItems();
    }
  }, [savedOwner, isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name && message.value1 !== undefined && message.owner) {
        Alert.alert(
          'New Item',
          `New Item: ${message.name}, Value: ${message.value1}, Owner: ${message.owner}`
        );
        if (savedOwner && message.owner.toLowerCase() === savedOwner.toLowerCase()) {
          loadItems();
        }
      }
    },
  });

  const loadSavedOwner = async () => {
    const owner = await getOwnerName();
    if (owner) {
      setSavedOwner(owner);
      setOwnerName(owner);
    }
  };

  const handleSaveOwner = async () => {
    const trimmed = ownerName.trim();
    if (!trimmed) {
      Alert.alert('Error', 'Owner name cannot be empty');
      return;
    }
    await saveOwnerName(trimmed);
    setSavedOwner(trimmed);
    Alert.alert('Success', 'Owner name saved');
  };

  const loadItems = async () => {
    if (!savedOwner) return;

    const cachedItems = await getLocalItems();
    const ownerItems = cachedItems.filter(
      (item) => item.owner.toLowerCase() === savedOwner.toLowerCase()
    );
    
    if (ownerItems.length > 0) {
      setItems(ownerItems);
    }

    if (!isOnline) {
      return;
    }

    setLoading(true);
    const response = await getItemsByOwner(savedOwner);
    setLoading(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      setItems(response.data);
      const allCached = await getLocalItems();
      const updatedItems = [...allCached];
      response.data.forEach((item) => {
        const index = updatedItems.findIndex((i) => i.id === item.id);
        if (index >= 0) {
          updatedItems[index] = item;
        } else {
          updatedItems.push(item);
        }
      });
      await saveItems(updatedItems);
    }
  };

  const handleSubmitItem = async () => {
    if (!savedOwner) {
      Alert.alert('Error', 'Please save owner name first');
      return;
    }

    const trimmedName = itemName.trim();
    const trimmedStatus = itemStatus.trim();
    const value1Num = parseFloat(itemValue1);

    if (!trimmedName) {
      Alert.alert('Error', 'Item name is required');
      return;
    }
    if (!trimmedStatus) {
      Alert.alert('Error', 'Status is required');
      return;
    }
    if (isNaN(value1Num)) {
      Alert.alert('Error', 'Value1 must be a number');
      return;
    }

    const newItem: Omit<Item, 'id' | 'value2'> = {
      name: trimmedName,
      status: trimmedStatus,
      owner: savedOwner,
      value1: value1Num,
    };

    if (isOnline) {
      setSubmitting(true);
      const response = await createItem(newItem);
      setSubmitting(false);

      if (response.error) {
        Alert.alert('Error', response.error);
        await savePendingItem({ ...newItem, id: Date.now(), value2: 0 });
        return;
      }

      if (response.data) {
        Alert.alert('Success', 'Item created');
        setItemName('');
        setItemStatus('');
        setItemValue1('');
        await loadItems();
      }
    } else {
      await savePendingItem({ ...newItem, id: Date.now(), value2: 0 });
      Alert.alert('Offline', 'Item saved offline. It will be synced when online.');
      setItemName('');
      setItemStatus('');
      setItemValue1('');
    }
  };

  const renderItem = ({ item }: { item: Item }) => (
    <ThemedView style={styles.itemContainer}>
      <ThemedText type="defaultSemiBold" style={styles.itemName}>
        {item.name}
      </ThemedText>
      <ThemedText style={styles.itemInfo}>
        Status: {item.status} | Value1: {item.value1} | Value2: {item.value2}
      </ThemedText>
    </ThemedView>
  );

  return (
    <ScrollView style={styles.container} contentContainerStyle={styles.contentContainer}>
      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>
          Owner Settings
        </ThemedText>
        <TextInput
          style={styles.input}
          placeholder="Enter owner name"
          value={ownerName}
          onChangeText={setOwnerName}
          autoCapitalize="none"
        />
        <TouchableOpacity style={styles.saveButton} onPress={handleSaveOwner}>
          <ThemedText style={styles.saveButtonText}>Save</ThemedText>
        </TouchableOpacity>
        {savedOwner && (
          <ThemedText style={styles.savedOwnerText}>Saved owner: {savedOwner}</ThemedText>
        )}
      </ThemedView>

      {savedOwner && (
        <ThemedView style={styles.section}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            Record Item
          </ThemedText>
          <TextInput
            style={styles.input}
            placeholder="Item name"
            value={itemName}
            onChangeText={setItemName}
            autoCapitalize="none"
          />
          <TextInput
            style={styles.input}
            placeholder="Status"
            value={itemStatus}
            onChangeText={setItemStatus}
            autoCapitalize="none"
          />
          <TextInput
            style={styles.input}
            placeholder="Value1"
            value={itemValue1}
            onChangeText={setItemValue1}
            keyboardType="numeric"
            autoCapitalize="none"
          />
          <ThemedText style={styles.ownerInfo}>Owner: {savedOwner}</ThemedText>
          <TouchableOpacity
            style={[styles.submitButton, submitting && styles.submitButtonDisabled]}
            onPress={handleSubmitItem}
            disabled={submitting}>
            {submitting ? (
              <ActivityIndicator size="small" color="#fff" />
            ) : (
              <ThemedText style={styles.submitButtonText}>Submit</ThemedText>
            )}
          </TouchableOpacity>
        </ThemedView>
      )}

      {savedOwner && (
        <ThemedView style={styles.section}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            My Items
          </ThemedText>
          {!isOnline ? (
            <View style={styles.offlineContainer}>
              <ThemedText style={styles.offlineMessage}>Offline - Showing cached data</ThemedText>
              <TouchableOpacity style={styles.retryButton} onPress={loadItems}>
                <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
              </TouchableOpacity>
            </View>
          ) : null}
          {loading ? (
            <LoadingSpinner visible={true} message="Loading items..." />
          ) : items.length === 0 ? (
            <ThemedText style={styles.emptyMessage}>No items found</ThemedText>
          ) : (
            <FlatList
              data={items}
              renderItem={renderItem}
              keyExtractor={(item) => item.id.toString()}
              scrollEnabled={false}
              ItemSeparatorComponent={() => <View style={styles.separator} />}
            />
          )}
        </ThemedView>
      )}
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  contentContainer: {
    padding: 16,
  },
  section: {
    marginBottom: 32,
  },
  sectionTitle: {
    marginBottom: 16,
  },
  input: {
    borderWidth: 1,
    borderColor: '#ccc',
    borderRadius: 8,
    padding: 12,
    fontSize: 16,
    marginBottom: 12,
    backgroundColor: '#fff',
  },
  saveButton: {
    backgroundColor: '#007AFF',
    paddingHorizontal: 24,
    paddingVertical: 12,
    borderRadius: 8,
    alignItems: 'center',
    marginBottom: 8,
  },
  saveButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
  savedOwnerText: {
    fontSize: 14,
    color: '#666',
    fontStyle: 'italic',
  },
  ownerInfo: {
    fontSize: 14,
    color: '#666',
    marginBottom: 12,
  },
  submitButton: {
    backgroundColor: '#34C759',
    paddingHorizontal: 24,
    paddingVertical: 12,
    borderRadius: 8,
    alignItems: 'center',
  },
  submitButtonDisabled: {
    opacity: 0.6,
  },
  submitButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
  offlineContainer: {
    padding: 16,
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
    marginBottom: 16,
    alignItems: 'center',
  },
  offlineMessage: {
    fontSize: 14,
    color: '#856404',
    marginBottom: 12,
    textAlign: 'center',
  },
  retryButton: {
    backgroundColor: '#FFC107',
    paddingHorizontal: 20,
    paddingVertical: 8,
    borderRadius: 6,
  },
  retryButtonText: {
    color: '#000',
    fontSize: 14,
    fontWeight: '600',
  },
  itemContainer: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  itemName: {
    fontSize: 16,
    marginBottom: 4,
  },
  itemInfo: {
    fontSize: 14,
    color: '#666',
  },
  emptyMessage: {
    fontSize: 14,
    color: '#666',
    fontStyle: 'italic',
    textAlign: 'center',
    padding: 16,
  },
  separator: {
    height: 8,
  },
});
