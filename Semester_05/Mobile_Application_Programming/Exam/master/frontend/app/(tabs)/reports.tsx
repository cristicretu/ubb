import React, { useState, useEffect, useMemo } from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  FlatList,
  Alert,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getAllItems } from '@/utils/api';
import { getLocalItems, saveItems } from '@/utils/storage';
import { Item } from '@/types/item';
import { useWebSocket } from '@/hooks/useWebSocket';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';

export default function ReportsSection() {
  const [items, setItems] = useState<Item[]>([]);
  const [loading, setLoading] = useState(false);
  const [isOnline, setIsOnline] = useState(false);
  const [isCached, setIsCached] = useState(false);

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
    loadItems();
  }, [isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.id && message.name) {
        Alert.alert(
          'New Item Added',
          `Name: ${message.name}\nStatus: ${message.status || 'N/A'}\nOwner: ${message.owner || 'N/A'}`
        );
        loadItems();
      }
    },
  });

  const loadItems = async () => {
    const cachedItems = await getLocalItems();
    if (cachedItems.length > 0) {
      setItems(cachedItems);
      setIsCached(true);
    }

    if (!isOnline) {
      return;
    }

    setLoading(true);
    setIsCached(false);
    const response = await getAllItems();
    setLoading(false);

    if (response.error) {
      log(`Error loading items: ${response.error}`, 'error');
      return;
    }

    if (response.data) {
      await saveItems(response.data);
      setItems(response.data);
      setIsCached(false);
    }
  };

  const top10ByValue2 = useMemo(() => {
    return [...items]
      .sort((a, b) => b.value2 - a.value2)
      .slice(0, 10);
  }, [items]);

  const top10ByValue1 = useMemo(() => {
    return [...items]
      .sort((a, b) => b.value1 - a.value1)
      .slice(0, 10);
  }, [items]);

  const top5OwnersByCount = useMemo(() => {
    const ownerCounts = items.reduce((acc, item) => {
      acc[item.owner] = (acc[item.owner] || 0) + 1;
      return acc;
    }, {} as Record<string, number>);

    return Object.entries(ownerCounts)
      .map(([owner, count]) => ({ owner, count }))
      .sort((a, b) => b.count - a.count)
      .slice(0, 5);
  }, [items]);

  const renderValue2Item = ({ item }: { item: Item }) => (
    <ThemedView style={styles.reportItem}>
      <ThemedText type="defaultSemiBold" style={styles.itemName}>
        {item.name}
      </ThemedText>
      <ThemedText style={styles.itemValue}>Value2: {item.value2}</ThemedText>
    </ThemedView>
  );

  const renderValue1Item = ({ item }: { item: Item }) => (
    <ThemedView style={styles.reportItem}>
      <ThemedText type="defaultSemiBold" style={styles.itemName}>
        {item.name}
      </ThemedText>
      <ThemedText style={styles.itemDetail}>
        Status: {item.status} | Value1: {item.value1} | Owner: {item.owner}
      </ThemedText>
    </ThemedView>
  );

  const renderOwnerItem = ({ item }: { item: { owner: string; count: number } }) => (
    <ThemedView style={styles.reportItem}>
      <ThemedText type="defaultSemiBold" style={styles.itemName}>
        {item.owner}
      </ThemedText>
      <ThemedText style={styles.itemValue}>Count: {item.count}</ThemedText>
    </ThemedView>
  );

  return (
    <ThemedView style={styles.container}>
      {loading ? (
        <LoadingSpinner visible={true} message="Loading reports..." />
      ) : (
        <ScrollView style={styles.scrollView} contentContainerStyle={styles.scrollContent}>
          {isCached && (
            <View style={styles.offlineContainer}>
              <ThemedText style={styles.offlineMessage}>
                Offline - Showing cached data
              </ThemedText>
            </View>
          )}

          <View style={styles.reportSection}>
            <ThemedText type="subtitle" style={styles.sectionTitle}>
              Top 10 by Value2 (Descending)
            </ThemedText>
            {top10ByValue2.length === 0 ? (
              <ThemedView style={styles.emptyContainer}>
                <ThemedText style={styles.emptyMessage}>No items found</ThemedText>
              </ThemedView>
            ) : (
              <FlatList
                data={top10ByValue2}
                renderItem={renderValue2Item}
                keyExtractor={(item) => `value2-${item.id}`}
                scrollEnabled={false}
                ItemSeparatorComponent={() => <View style={styles.separator} />}
              />
            )}
          </View>

          <View style={styles.reportSection}>
            <ThemedText type="subtitle" style={styles.sectionTitle}>
              Top 10 by Value1 (Descending)
            </ThemedText>
            {top10ByValue1.length === 0 ? (
              <ThemedView style={styles.emptyContainer}>
                <ThemedText style={styles.emptyMessage}>No items found</ThemedText>
              </ThemedView>
            ) : (
              <FlatList
                data={top10ByValue1}
                renderItem={renderValue1Item}
                keyExtractor={(item) => `value1-${item.id}`}
                scrollEnabled={false}
                ItemSeparatorComponent={() => <View style={styles.separator} />}
              />
            )}
          </View>

          <View style={styles.reportSection}>
            <ThemedText type="subtitle" style={styles.sectionTitle}>
              Top 5 Owners by Count
            </ThemedText>
            {top5OwnersByCount.length === 0 ? (
              <ThemedView style={styles.emptyContainer}>
                <ThemedText style={styles.emptyMessage}>No owners found</ThemedText>
              </ThemedView>
            ) : (
              <FlatList
                data={top5OwnersByCount}
                renderItem={renderOwnerItem}
                keyExtractor={(item) => `owner-${item.owner}`}
                scrollEnabled={false}
                ItemSeparatorComponent={() => <View style={styles.separator} />}
              />
            )}
          </View>
        </ScrollView>
      )}
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
    textAlign: 'center',
  },
  reportSection: {
    marginBottom: 24,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: '600',
    marginBottom: 12,
    color: '#000',
  },
  reportItem: {
    padding: 12,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  itemName: {
    fontSize: 16,
    marginBottom: 4,
  },
  itemValue: {
    fontSize: 14,
    color: '#666',
  },
  itemDetail: {
    fontSize: 14,
    color: '#666',
  },
  separator: {
    height: 8,
  },
  emptyContainer: {
    padding: 20,
    alignItems: 'center',
  },
  emptyMessage: {
    fontSize: 14,
    color: '#999',
    textAlign: 'center',
  },
});
