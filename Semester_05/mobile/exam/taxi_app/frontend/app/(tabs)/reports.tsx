import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  FlatList,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getAllCabs } from '@/utils/api';
import { getLocalCabs, saveCabs } from '@/utils/storage';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { Cab } from '@/types/cab';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';

export default function ReportsSection() {
  const [cabs, setCabs] = useState<Cab[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isOnline, setIsOnline] = useState(true);

  useEffect(() => {
    const unsubscribe = NetInfo.addEventListener((state) => {
      const online = state.isConnected ?? false;
      setIsOnline(online);
      log(`Network: ${online ? 'online' : 'offline'}`, 'info');
    });

    NetInfo.fetch().then((state) => setIsOnline(state.isConnected ?? false));
    return () => unsubscribe();
  }, []);

  useEffect(() => {
    loadCabs();
  }, []);

  const loadCabs = async () => {
    setIsLoading(true);
    log('Loading cabs for reports...', 'info');

    try {
      // Try cached cabs first
      const localCabs = await getLocalCabs();
      
      if (localCabs.length > 0) {
        setCabs(localCabs);
        log(`Loaded ${localCabs.length} cached cabs`, 'success');
      }

      // Check network status and fetch from API if online
      const netState = await NetInfo.fetch();
      const online = netState.isConnected ?? false;
      setIsOnline(online);

      if (online) {
        const response = await getAllCabs();
        if (response.error) {
          log(`Error fetching cabs: ${response.error}`, 'error');
          // Keep using cached cabs if available
        } else if (response.data) {
          setCabs(response.data);
          await saveCabs(response.data);
          log(`Loaded ${response.data.length} cabs from API`, 'success');
        }
      }
    } catch (error) {
      const msg = error instanceof Error ? error.message : 'Unknown error';
      log(`Error loading cabs: ${msg}`, 'error');
    } finally {
      setIsLoading(false);
    }
  };

  // Top 10 cabs by SIZE (descending)
  const top10BySize = React.useMemo(() => {
    return [...cabs]
      .sort((a, b) => b.size - a.size)
      .slice(0, 10);
  }, [cabs]);

  // Top 10 drivers by NUMBER OF CABS (descending)
  const top10Drivers = React.useMemo(() => {
    const driverCounts = new Map<string, number>();
    
    cabs.forEach((cab) => {
      const count = driverCounts.get(cab.driver) || 0;
      driverCounts.set(cab.driver, count + 1);
    });

    return Array.from(driverCounts.entries())
      .map(([driver, count]) => ({ driver, count }))
      .sort((a, b) => b.count - a.count)
      .slice(0, 10);
  }, [cabs]);

  // Top 5 biggest cabs by CAPACITY (descending)
  const top5ByCapacity = React.useMemo(() => {
    return [...cabs]
      .sort((a, b) => b.capacity - a.capacity)
      .slice(0, 5);
  }, [cabs]);

  const renderTop10BySizeItem = ({ item }: { item: Cab }) => (
    <ThemedView style={styles.reportItem}>
      <ThemedText type="defaultSemiBold" style={styles.itemName}>{item.name}</ThemedText>
      <ThemedText style={styles.itemDetail}>Status: {item.status}</ThemedText>
      <ThemedText style={styles.itemDetail}>Size: {item.size}</ThemedText>
      <ThemedText style={styles.itemDetail}>Driver: {item.driver}</ThemedText>
    </ThemedView>
  );

  const renderTop10DriversItem = ({ item }: { item: { driver: string; count: number } }) => (
    <ThemedView style={styles.reportItem}>
      <ThemedText type="defaultSemiBold" style={styles.itemName}>{item.driver}</ThemedText>
      <ThemedText style={styles.itemDetail}>Count of cabs: {item.count}</ThemedText>
    </ThemedView>
  );

  const renderTop5ByCapacityItem = ({ item }: { item: Cab }) => (
    <ThemedView style={styles.reportItem}>
      <ThemedText type="defaultSemiBold" style={styles.itemName}>{item.name}</ThemedText>
      <ThemedText style={styles.itemDetail}>Capacity: {item.capacity}</ThemedText>
    </ThemedView>
  );

  return (
    <ThemedView style={styles.container}>
      <ScrollView
        style={styles.scrollView}
        contentContainerStyle={styles.scrollContent}
      >
        {isLoading ? (
          <LoadingSpinner visible={true} message="Loading reports..." />
        ) : (
          <>
            {!isOnline && (
              <ThemedView style={styles.offlineBanner}>
                <ThemedText style={styles.offlineBannerText}>
                  Showing cached data. You are offline.
                </ThemedText>
              </ThemedView>
            )}

            {/* Section A: Top 10 cabs by SIZE */}
            <ThemedView style={styles.section}>
              <ThemedText type="title" style={styles.sectionTitle}>
                Top 10 Cabs by Size
              </ThemedText>
              {top10BySize.length === 0 ? (
                <ThemedView style={styles.emptyContainer}>
                  <ThemedText style={styles.emptyText}>No cabs available</ThemedText>
                </ThemedView>
              ) : (
                <FlatList
                  data={top10BySize}
                  renderItem={renderTop10BySizeItem}
                  keyExtractor={(item, index) => `size-${item.id}-${index}`}
                  scrollEnabled={false}
                  contentContainerStyle={styles.listContent}
                />
              )}
            </ThemedView>

            {/* Section B: Top 10 drivers by NUMBER OF CABS */}
            <ThemedView style={styles.section}>
              <ThemedText type="title" style={styles.sectionTitle}>
                Top 10 Drivers by Number of Cabs
              </ThemedText>
              {top10Drivers.length === 0 ? (
                <ThemedView style={styles.emptyContainer}>
                  <ThemedText style={styles.emptyText}>No drivers available</ThemedText>
                </ThemedView>
              ) : (
                <FlatList
                  data={top10Drivers}
                  renderItem={renderTop10DriversItem}
                  keyExtractor={(item, index) => `driver-${item.driver}-${index}`}
                  scrollEnabled={false}
                  contentContainerStyle={styles.listContent}
                />
              )}
            </ThemedView>

            {/* Section C: Top 5 biggest cabs by CAPACITY */}
            <ThemedView style={styles.section}>
              <ThemedText type="title" style={styles.sectionTitle}>
                Top 5 Biggest Cabs by Capacity
              </ThemedText>
              {top5ByCapacity.length === 0 ? (
                <ThemedView style={styles.emptyContainer}>
                  <ThemedText style={styles.emptyText}>No cabs available</ThemedText>
                </ThemedView>
              ) : (
                <FlatList
                  data={top5ByCapacity}
                  renderItem={renderTop5ByCapacityItem}
                  keyExtractor={(item, index) => `capacity-${item.id}-${index}`}
                  scrollEnabled={false}
                  contentContainerStyle={styles.listContent}
                />
              )}
            </ThemedView>
          </>
        )}
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
    marginBottom: 32,
  },
  sectionTitle: {
    marginBottom: 16,
    fontSize: 24,
  },
  offlineBanner: {
    padding: 12,
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
    marginBottom: 16,
    borderWidth: 1,
    borderColor: '#FFE69C',
  },
  offlineBannerText: {
    fontSize: 14,
    color: '#856404',
    textAlign: 'center',
  },
  listContent: {
    paddingBottom: 16,
  },
  reportItem: {
    padding: 16,
    marginBottom: 12,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#ddd',
    backgroundColor: '#f9f9f9',
  },
  itemName: {
    fontSize: 18,
    marginBottom: 8,
  },
  itemDetail: {
    fontSize: 14,
    marginBottom: 4,
    color: '#666',
  },
  emptyContainer: {
    padding: 40,
    alignItems: 'center',
  },
  emptyText: {
    fontSize: 16,
    color: '#999',
  },
});
