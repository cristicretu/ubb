import React, { useState, useEffect, useCallback } from 'react';
import { View, StyleSheet, FlatList, Alert, TouchableOpacity } from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { useFocusEffect } from '@react-navigation/native';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import LoadingSpinner from '@/components/LoadingSpinner';
import { getAllPayments } from '@/utils/api';
import { Payment } from '@/types/payment';
import { useWebSocket } from '@/hooks/useWebSocket';

interface CategoryTotal {
  category: string;
  total: number;
}

export default function InsightsSection() {
  const [isOnline, setIsOnline] = useState(false);
  const [loading, setLoading] = useState(false);
  const [topCategories, setTopCategories] = useState<CategoryTotal[]>([]);

  useEffect(() => {
    const unsubscribe = NetInfo.addEventListener((state) => {
      const online = state.isConnected ?? false;
      setIsOnline(online);
    });
    NetInfo.fetch().then((state) => setIsOnline(state.isConnected ?? false));
    return () => unsubscribe();
  }, []);

  useEffect(() => {
    if (isOnline) {
      loadTopCategories();
    }
  }, [isOnline]);

  useFocusEffect(
    useCallback(() => {
      if (isOnline) {
        loadTopCategories();
      }
    }, [isOnline])
  );

  useWebSocket({
    onMessage: (message: any) => {
      if (message.date && message.amount !== undefined && message.type) {
        if (isOnline) {
          loadTopCategories();
        }
      }
    },
  });

  const loadTopCategories = async () => {
    if (!isOnline) {
      return;
    }

    setLoading(true);
    const response = await getAllPayments();
    setLoading(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      const categoryMap = new Map<string, number>();

      response.data.forEach((payment: Payment) => {
        const currentTotal = categoryMap.get(payment.category) || 0;
        categoryMap.set(payment.category, currentTotal + payment.amount);
      });

      const categories: CategoryTotal[] = Array.from(categoryMap.entries())
        .map(([category, total]) => ({ category, total }))
        .sort((a, b) => b.total - a.total)
        .slice(0, 3);

      setTopCategories(categories);
    }
  };

  const formatCurrency = (amount: number) => {
    return `$${amount.toFixed(2).replace(/\B(?=(\d{3})+(?!\d))/g, ',')}`;
  };

  const renderItem = ({ item }: { item: CategoryTotal }) => {
    return (
      <ThemedView style={styles.categoryItem}>
        <ThemedText type="defaultSemiBold" style={styles.categoryName}>
          {item.category}
        </ThemedText>
        <ThemedText style={styles.categoryTotal}>
          Total: {formatCurrency(item.total)}
        </ThemedText>
      </ThemedView>
    );
  };

  return (
    <View style={styles.container}>
      <View style={styles.header}>
        <ThemedText type="subtitle" style={styles.title}>
          Top Departments
        </ThemedText>
        {!isOnline && (
          <View style={styles.offlineBanner}>
            <ThemedText style={styles.offlineText}>Offline</ThemedText>
            <TouchableOpacity onPress={loadTopCategories} style={styles.retryButton}>
              <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
            </TouchableOpacity>
          </View>
        )}
      </View>
      {loading ? (
        <LoadingSpinner visible={true} />
      ) : topCategories.length === 0 ? (
        <ThemedText style={styles.emptyMessage}>No categories found</ThemedText>
      ) : (
        <FlatList
          data={topCategories}
          renderItem={renderItem}
          keyExtractor={(item) => item.category}
          contentContainerStyle={styles.listContent}
          ItemSeparatorComponent={() => <View style={styles.separator} />}
        />
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  header: {
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#E0E0E0',
  },
  title: {
    marginBottom: 8,
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
  categoryItem: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  categoryName: {
    fontSize: 18,
    marginBottom: 8,
  },
  categoryTotal: {
    fontSize: 16,
    color: '#666',
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
