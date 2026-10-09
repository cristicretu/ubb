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
import { getPayments, getPaymentById, createPayment, deletePayment } from '@/utils/api';
import { savePayments, getLocalPayments, savePaymentDetail, getPaymentDetail } from '@/utils/storage';
import { Payment } from '@/types/payment';
import { useWebSocket } from '@/hooks/useWebSocket';

export default function MainSection() {
  const [isOnline, setIsOnline] = useState(false);
  const [date, setDate] = useState('');
  const [amount, setAmount] = useState('');
  const [type, setType] = useState('');
  const [category, setCategory] = useState('');
  const [description, setDescription] = useState('');
  const [payments, setPayments] = useState<Payment[]>([]);
  const [loading, setLoading] = useState(false);
  const [submitting, setSubmitting] = useState(false);
  const [loadingDetails, setLoadingDetails] = useState<number | null>(null);
  const [isOfflineMode, setIsOfflineMode] = useState(false);
  const justSubmittedId = useRef<number | null>(null);

  const wasOffline = useRef(false);

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
      loadPayments();
    }
    wasOffline.current = !isOnline;
  }, [isOnline]);

  useEffect(() => {
    loadPayments();
  }, []);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.date && message.amount !== undefined && message.type) {
        if (message.id === justSubmittedId.current) {
          justSubmittedId.current = null;
          return;
        }
        Alert.alert(
          'New Payment',
          `Date: ${message.date}\nAmount: ${message.amount}\nType: ${message.type}`
        );
        loadPayments();
      }
    },
  });

  const loadPayments = async () => {
    setLoading(true);
    const response = await getPayments();
    setLoading(false);

    if (response.error) {
      const cached = await getLocalPayments();
      setPayments(cached);
      setIsOfflineMode(true);
    } else if (response.data) {
      setPayments(response.data);
      setIsOfflineMode(false);
      await savePayments(response.data);
    }
  };

  const handleSubmit = async () => {
    if (!date || !amount || !type || !category || !description) {
      Alert.alert('Error', 'Please fill all fields');
      return;
    }

    const amountNum = parseFloat(amount);
    if (isNaN(amountNum) || amountNum <= 0) {
      Alert.alert('Error', 'Amount must be a positive number');
      return;
    }

    if (!isOnline) {
      Alert.alert('Error', 'Cannot create payment while offline');
      return;
    }

    setSubmitting(true);
    const newPayment: Omit<Payment, 'id'> = {
      date,
      amount: amountNum,
      type,
      category,
      description,
    };

    const response = await createPayment(newPayment);
    setSubmitting(false);

    if (response.error) {
      if (response.error.includes('Network') || response.error.includes('network')) {
        setIsOfflineMode(true);
      }
      Alert.alert('Error', response.error);
    } else if (response.data) {
      justSubmittedId.current = response.data.id;
      setDate('');
      setAmount('');
      setType('');
      setCategory('');
      setDescription('');
      Alert.alert('Success', 'Payment created');
      loadPayments();
    }
  };

  const handlePaymentPress = async (payment: Payment) => {
    setLoadingDetails(payment.id);
    let paymentData: Payment | null = null;

    if (isOnline) {
      const response = await getPaymentById(payment.id);
      if (response.error) {
        const cached = await getPaymentDetail(payment.id);
        if (cached) {
          paymentData = cached;
        } else {
          setLoadingDetails(null);
          Alert.alert('Error', response.error);
          return;
        }
      } else if (response.data) {
        paymentData = response.data;
        await savePaymentDetail(payment.id, response.data);
      }
    } else {
      const cached = await getPaymentDetail(payment.id);
      if (cached) {
        paymentData = cached;
      } else {
        setLoadingDetails(null);
        Alert.alert('Offline', 'Payment details not available in cache');
        return;
      }
    }

    setLoadingDetails(null);

    if (paymentData) {
      Alert.alert(
        'Payment Details',
        `ID: ${paymentData.id}\nDate: ${paymentData.date}\nAmount: ${paymentData.amount}\nType: ${paymentData.type}\nCategory: ${paymentData.category}\nDescription: ${paymentData.description}`
      );
    }
  };

  const handleDelete = async (id: number) => {
    if (!isOnline) {
      Alert.alert('Error', 'Cannot delete payment while offline');
      return;
    }

    Alert.alert(
      'Delete Payment',
      'Are you sure you want to delete this payment?',
      [
        { text: 'Cancel', style: 'cancel' },
        {
          text: 'Delete',
          style: 'destructive',
          onPress: async () => {
            setLoading(true);
            const response = await deletePayment(id);
            setLoading(false);

            if (response.error) {
              if (response.error.includes('Network') || response.error.includes('network')) {
                setIsOfflineMode(true);
              }
              Alert.alert('Error', response.error);
            } else {
              Alert.alert('Success', 'Payment deleted');
              loadPayments();
            }
          },
        },
      ]
    );
  };

  const renderItem = ({ item }: { item: Payment }) => {
    const isLoading = loadingDetails === item.id;
    return (
      <TouchableOpacity onPress={() => handlePaymentPress(item)} disabled={isLoading}>
        <ThemedView style={styles.paymentItem}>
          <View style={styles.paymentHeader}>
            <ThemedText type="defaultSemiBold" style={styles.paymentDate}>
              {item.date}
            </ThemedText>
            <TouchableOpacity
              onPress={() => handleDelete(item.id)}
              disabled={!isOnline}
              style={[styles.deleteButton, !isOnline && styles.deleteButtonDisabled]}>
              <ThemedText style={styles.deleteButtonText}>Delete</ThemedText>
            </TouchableOpacity>
          </View>
          <ThemedText style={styles.paymentAmount}>Amount: {item.amount}</ThemedText>
          <ThemedText style={styles.paymentType}>Type: {item.type}</ThemedText>
          <ThemedText style={styles.paymentCategory}>Category: {item.category}</ThemedText>
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
          Add Payment
        </ThemedText>
        <TextInput
          style={styles.input}
          placeholder="Date"
          value={date}
          onChangeText={setDate}
          autoCapitalize="none"
        />
        <TextInput
          style={styles.input}
          placeholder="Amount"
          value={amount}
          onChangeText={setAmount}
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
        <TextInput
          style={styles.input}
          placeholder="Category"
          value={category}
          onChangeText={setCategory}
          autoCapitalize="none"
        />
        <TextInput
          style={styles.input}
          placeholder="Description"
          value={description}
          onChangeText={setDescription}
          autoCapitalize="none"
          multiline
        />
        <TouchableOpacity
          style={[styles.submitButton, (submitting || !isOnline) && styles.submitButtonDisabled]}
          onPress={handleSubmit}
          disabled={submitting || !isOnline}>
          {submitting ? (
            <ActivityIndicator size="small" color="#fff" />
          ) : (
            <ThemedText style={styles.submitButtonText}>Submit Payment</ThemedText>
          )}
        </TouchableOpacity>
      </ScrollView>

      <View style={styles.paymentsContainer}>
        <View style={styles.paymentsHeader}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            Payments
          </ThemedText>
          {(!isOnline || isOfflineMode) && (
            <View style={styles.offlineBanner}>
              <ThemedText style={styles.offlineText}>
                {!isOnline ? 'You are offline' : 'Showing cached data'}
              </ThemedText>
              <TouchableOpacity onPress={loadPayments} style={styles.retryButton}>
                <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
              </TouchableOpacity>
            </View>
          )}
        </View>
        {loading ? (
          <LoadingSpinner visible={true} message="Loading payments..." />
        ) : payments.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>No payments</ThemedText>
        ) : (
          <FlatList
            data={payments}
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
  paymentsContainer: {
    flex: 1,
  },
  paymentsHeader: {
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
  paymentItem: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  paymentHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 8,
  },
  paymentDate: {
    fontSize: 18,
  },
  deleteButton: {
    paddingHorizontal: 12,
    paddingVertical: 6,
    backgroundColor: '#FF3B30',
    borderRadius: 4,
  },
  deleteButtonDisabled: {
    opacity: 0.5,
  },
  deleteButtonText: {
    color: '#fff',
    fontSize: 12,
    fontWeight: '600',
  },
  paymentAmount: {
    fontSize: 14,
    color: '#666',
    marginBottom: 4,
  },
  paymentType: {
    fontSize: 14,
    color: '#666',
    marginBottom: 4,
  },
  paymentCategory: {
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
