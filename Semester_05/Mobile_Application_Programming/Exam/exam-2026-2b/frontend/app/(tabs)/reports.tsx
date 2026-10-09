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

interface MonthlyTotal {
  month: string;
  total: number;
}

export default function ReportsSection() {
  const [isOnline, setIsOnline] = useState(false);
  const [loading, setLoading] = useState(false);
  const [monthlyTotals, setMonthlyTotals] = useState<MonthlyTotal[]>([]);

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
      loadPayments();
    }
  }, [isOnline]);

  useFocusEffect(
    useCallback(() => {
      if (isOnline) {
        loadPayments();
      }
    }, [isOnline])
  );

  useWebSocket({
    onMessage: (message: any) => {
      if (message.date && message.amount !== undefined && message.type) {
        if (isOnline) {
          loadPayments();
        }
      }
    },
  });

  const loadPayments = async () => {
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
      const totals = calculateMonthlyTotals(response.data);
      setMonthlyTotals(totals);
    }
  };

  const calculateMonthlyTotals = (payments: Payment[]): MonthlyTotal[] => {
    const monthMap = new Map<string, number>();

    payments.forEach((payment) => {
      const date = new Date(payment.date);
      const monthKey = `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, '0')}`;
      const currentTotal = monthMap.get(monthKey) || 0;
      monthMap.set(monthKey, currentTotal + payment.amount);
    });

    const totals: MonthlyTotal[] = Array.from(monthMap.entries()).map(([month, total]) => ({
      month,
      total,
    }));

    return totals.sort((a, b) => b.total - a.total);
  };

  const formatCurrency = (amount: number): string => {
    return `$${amount.toFixed(2).replace(/\B(?=(\d{3})+(?!\d))/g, ',')}`;
  };

  const renderItem = ({ item }: { item: MonthlyTotal }) => (
    <ThemedView style={styles.item}>
      <ThemedText type="defaultSemiBold" style={styles.month}>
        {item.month}
      </ThemedText>
      <ThemedText style={styles.total}>Total: {formatCurrency(item.total)}</ThemedText>
    </ThemedView>
  );

  return (
    <View style={styles.container}>
      <ThemedText type="subtitle" style={styles.title}>
        Monthly Payroll Analysis
      </ThemedText>
      {!isOnline && (
        <View style={styles.offlineBanner}>
          <ThemedText style={styles.offlineText}>You are offline</ThemedText>
          <TouchableOpacity onPress={loadPayments} style={styles.retryButton}>
            <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
          </TouchableOpacity>
        </View>
      )}
      {loading ? (
        <LoadingSpinner visible={true} />
      ) : monthlyTotals.length === 0 ? (
        <ThemedText style={styles.emptyMessage}>No payment data available</ThemedText>
      ) : (
        <FlatList
          data={monthlyTotals}
          renderItem={renderItem}
          keyExtractor={(item) => item.month}
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
    padding: 16,
  },
  title: {
    marginBottom: 16,
  },
  offlineBanner: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 16,
    padding: 12,
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
  },
  offlineText: {
    fontSize: 14,
    color: '#856404',
  },
  retryButton: {
    paddingHorizontal: 16,
    paddingVertical: 8,
    backgroundColor: '#007AFF',
    borderRadius: 4,
  },
  retryButtonText: {
    color: '#fff',
    fontSize: 14,
    fontWeight: '600',
  },
  listContent: {
    paddingBottom: 16,
  },
  item: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  month: {
    fontSize: 18,
    marginBottom: 4,
  },
  total: {
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
    marginTop: 32,
  },
});
