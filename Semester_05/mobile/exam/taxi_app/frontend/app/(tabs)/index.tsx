import React, { useState, useEffect, useCallback, useRef } from 'react';
import {
  View,
  TextInput,
  StyleSheet,
  ScrollView,
  FlatList,
  Alert,
  TouchableOpacity,
  RefreshControl,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { createCab, getAllCabs } from '@/utils/api';
import { saveCabs, getLocalCabs, savePendingCab } from '@/utils/storage';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { useWebSocket } from '@/hooks/useWebSocket';
import { Cab } from '@/types/cab';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';

export default function RegistrationSection() {
  const [name, setName] = useState('');
  const [status, setStatus] = useState('');
  const [size, setSize] = useState('');
  const [driver, setDriver] = useState('');
  const [color, setColor] = useState('');
  const [capacity, setCapacity] = useState('');
  const [cabs, setCabs] = useState<Cab[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isOnline, setIsOnline] = useState(true);
  const [refreshing, setRefreshing] = useState(false);
  const justSubmittedId = useRef<number | null>(null);

  const handleWebSocketMessage = useCallback((message: any) => {
    log('WebSocket message received', 'info');
    
    if (message?.name && message?.size !== undefined && message?.driver) {
      const newCab: Cab = {
        id: message.id || Date.now(),
        name: message.name,
        status: message.status || '',
        size: message.size,
        driver: message.driver,
        color: message.color || '',
        capacity: message.capacity || 0,
      };

      if (justSubmittedId.current === newCab.id) {
        justSubmittedId.current = null;
        return;
      }

      Alert.alert('New Cab', `Name: ${newCab.name}\nSize: ${newCab.size}\nDriver: ${newCab.driver}`);

      setCabs((prev) => {
        if (prev.some((c) => c.id === newCab.id)) return prev;
        return [newCab, ...prev];
      });

      log(`New cab via WebSocket: ${newCab.name}`, 'success');
    }
  }, []);

  useWebSocket({ onMessage: handleWebSocketMessage });

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
    log('Loading cabs...', 'info');

    try {
      if (isOnline) {
        const response = await getAllCabs();
        if (response.error) {
          log(`Error: ${response.error}`, 'error');
          const localCabs = await getLocalCabs();
          setCabs(localCabs);
          Alert.alert('Error', response.error);
        } else if (response.data) {
          setCabs(response.data);
          await saveCabs(response.data);
          log(`Loaded ${response.data.length} cabs`, 'success');
        }
      } else {
        const localCabs = await getLocalCabs();
        setCabs(localCabs);
        log(`Loaded ${localCabs.length} cached cabs`, 'info');
      }
    } catch (error) {
      const msg = error instanceof Error ? error.message : 'Unknown error';
      log(`Error: ${msg}`, 'error');
      Alert.alert('Error', msg);
    } finally {
      setIsLoading(false);
    }
  };

  const onRefresh = useCallback(async () => {
    setRefreshing(true);
    await loadCabs();
    setRefreshing(false);
  }, [isOnline]);

  const handleSubmit = async () => {
    if (!name.trim()) return Alert.alert('Error', 'Enter cab name');
    if (!status.trim()) return Alert.alert('Error', 'Enter status');
    if (!size.trim() || isNaN(Number(size))) return Alert.alert('Error', 'Enter valid size');
    if (!driver.trim()) return Alert.alert('Error', 'Enter driver name');
    if (!color.trim()) return Alert.alert('Error', 'Enter color');
    if (!capacity.trim() || isNaN(Number(capacity))) return Alert.alert('Error', 'Enter valid capacity');

    setIsSubmitting(true);
    log(`Submitting: ${name}`, 'info');

    const cabData = {
      name: name.trim(),
      status: status.trim(),
      size: Number(size),
      driver: driver.trim(),
      color: color.trim(),
      capacity: Number(capacity),
    };

    try {
      if (isOnline) {
        const response = await createCab(cabData as Cab);
        if (response.error) {
          log(`Error: ${response.error}`, 'error');
          Alert.alert('Error', response.error);
        } else if (response.data) {
          justSubmittedId.current = response.data.id;
          setCabs((prev) => [response.data!, ...prev]);
          await saveCabs([response.data, ...cabs]);
          Alert.alert('Success', 'Cab created!');
          log(`Created: ${response.data.name}`, 'success');
          setName(''); setStatus(''); setSize(''); setDriver(''); setColor(''); setCapacity('');
        }
      } else {
        const pendingCab: Cab = { id: Date.now(), ...cabData };
        await savePendingCab(pendingCab);
        setCabs((prev) => [pendingCab, ...prev]);
        Alert.alert('Saved Offline', 'Will sync when online.');
        log(`Saved offline: ${pendingCab.name}`, 'success');
        setName(''); setStatus(''); setSize(''); setDriver(''); setColor(''); setCapacity('');
      }
    } catch (error) {
      const msg = error instanceof Error ? error.message : 'Unknown error';
      log(`Error: ${msg}`, 'error');
      Alert.alert('Error', msg);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleRetry = async () => {
    if (isOnline) await loadCabs();
    else Alert.alert('Offline', 'Check your connection.');
  };

  const renderCabItem = ({ item }: { item: Cab }) => (
    <ThemedView style={styles.cabItem}>
      <ThemedText type="defaultSemiBold" style={styles.cabName}>{item.name}</ThemedText>
      <ThemedText style={styles.cabDetail}>ID: {item.id}</ThemedText>
      <ThemedText style={styles.cabDetail}>Status: {item.status}</ThemedText>
      <ThemedText style={styles.cabDetail}>Size: {item.size} passengers</ThemedText>
      <ThemedText style={styles.cabDetail}>Driver: {item.driver}</ThemedText>
      <ThemedText style={styles.cabDetail}>Color: {item.color}</ThemedText>
      <ThemedText style={styles.cabDetail}>Capacity: {item.capacity}</ThemedText>
    </ThemedView>
  );

  return (
    <ThemedView style={styles.container}>
      <ScrollView style={styles.scrollView} contentContainerStyle={styles.scrollContent} refreshControl={<RefreshControl refreshing={refreshing} onRefresh={onRefresh} />}>
        
        <ThemedView style={styles.section}>
          <ThemedText type="title" style={styles.sectionTitle}>Record New Cab</ThemedText>

          <ThemedText style={styles.label}>Name</ThemedText>
          <TextInput style={styles.input} value={name} onChangeText={setName} placeholder="Cab name" placeholderTextColor="#999" autoCapitalize="none" editable={!isSubmitting} />

          <ThemedText style={styles.label}>Status</ThemedText>
          <TextInput style={styles.input} value={status} onChangeText={setStatus} placeholder="new, working, damaged, private" placeholderTextColor="#999" autoCapitalize="none" editable={!isSubmitting} />

          <ThemedText style={styles.label}>Size (passengers)</ThemedText>
          <TextInput style={styles.input} value={size} onChangeText={setSize} placeholder="Number of passengers" placeholderTextColor="#999" keyboardType="numeric" editable={!isSubmitting} />

          <ThemedText style={styles.label}>Driver</ThemedText>
          <TextInput style={styles.input} value={driver} onChangeText={setDriver} placeholder="Driver name" placeholderTextColor="#999" autoCapitalize="none" editable={!isSubmitting} />

          <ThemedText style={styles.label}>Color</ThemedText>
          <TextInput style={styles.input} value={color} onChangeText={setColor} placeholder="Cab color" placeholderTextColor="#999" autoCapitalize="none" editable={!isSubmitting} />

          <ThemedText style={styles.label}>Capacity (trunk)</ThemedText>
          <TextInput style={styles.input} value={capacity} onChangeText={setCapacity} placeholder="Trunk volume" placeholderTextColor="#999" keyboardType="numeric" editable={!isSubmitting} />

          <TouchableOpacity style={[styles.submitButton, isSubmitting && styles.submitButtonDisabled]} onPress={handleSubmit} disabled={isSubmitting}>
            {isSubmitting ? (
              <View style={styles.submitButtonLoading}>
                <ActivityIndicator size="small" color="#fff" />
                <ThemedText style={styles.submitButtonText}>Submitting...</ThemedText>
              </View>
            ) : (
              <ThemedText style={styles.submitButtonText}>{isOnline ? 'Submit' : 'Save Offline'}</ThemedText>
            )}
          </TouchableOpacity>

          {!isOnline && <ThemedText style={styles.offlineIndicator}>You are offline. Cab will be saved locally.</ThemedText>}
        </ThemedView>

        <ThemedView style={styles.section}>
          <ThemedText type="title" style={styles.sectionTitle}>All Cabs</ThemedText>

          {isLoading ? (
            <LoadingSpinner visible={true} message="Loading cabs..." />
          ) : !isOnline && cabs.length === 0 ? (
            <ThemedView style={styles.offlineContainer}>
              <ThemedText style={styles.offlineMessage}>You are offline. No cached cabs.</ThemedText>
              <TouchableOpacity style={styles.retryButton} onPress={handleRetry}>
                <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
              </TouchableOpacity>
            </ThemedView>
          ) : cabs.length === 0 ? (
            <ThemedView style={styles.emptyContainer}>
              <ThemedText style={styles.emptyText}>No cabs found</ThemedText>
            </ThemedView>
          ) : (
            <>
              {!isOnline && (
                <ThemedView style={styles.offlineBanner}>
                  <ThemedText style={styles.offlineBannerText}>Showing cached cabs.</ThemedText>
                  <TouchableOpacity style={styles.retryButtonSmall} onPress={handleRetry}>
                    <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
                  </TouchableOpacity>
                </ThemedView>
              )}
              <FlatList data={cabs} renderItem={renderCabItem} keyExtractor={(item, index) => `${item.id}-${index}`} scrollEnabled={false} />
            </>
          )}
        </ThemedView>
      </ScrollView>
    </ThemedView>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1 },
  scrollView: { flex: 1 },
  scrollContent: { padding: 16 },
  section: { marginBottom: 24 },
  sectionTitle: { marginBottom: 16, fontSize: 24 },
  label: { fontSize: 14, fontWeight: '600', marginBottom: 8, marginTop: 12 },
  input: { borderWidth: 1, borderColor: '#ddd', borderRadius: 8, padding: 12, fontSize: 16, backgroundColor: '#fff', color: '#000' },
  submitButton: { backgroundColor: '#007AFF', borderRadius: 8, padding: 16, alignItems: 'center', marginTop: 20 },
  submitButtonDisabled: { opacity: 0.6 },
  submitButtonText: { color: '#fff', fontSize: 16, fontWeight: '600' },
  submitButtonLoading: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  offlineIndicator: { marginTop: 12, fontSize: 14, color: '#FF9500', textAlign: 'center' },
  offlineContainer: { padding: 20, alignItems: 'center', backgroundColor: '#FFF3CD', borderRadius: 8 },
  offlineMessage: { fontSize: 16, marginBottom: 16, textAlign: 'center', color: '#856404' },
  offlineBanner: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', padding: 12, backgroundColor: '#FFF3CD', borderRadius: 8, marginBottom: 12 },
  offlineBannerText: { flex: 1, fontSize: 14, color: '#856404' },
  retryButton: { backgroundColor: '#007AFF', borderRadius: 8, paddingHorizontal: 24, paddingVertical: 12 },
  retryButtonSmall: { backgroundColor: '#007AFF', borderRadius: 6, paddingHorizontal: 16, paddingVertical: 8 },
  retryButtonText: { color: '#fff', fontSize: 14, fontWeight: '600' },
  emptyContainer: { padding: 40, alignItems: 'center' },
  emptyText: { fontSize: 16, color: '#999' },
  cabItem: { padding: 16, marginBottom: 12, borderRadius: 8, borderWidth: 1, borderColor: '#ddd', backgroundColor: '#f9f9f9' },
  cabName: { fontSize: 18, marginBottom: 8 },
  cabDetail: { fontSize: 14, marginBottom: 4, color: '#666' },
});
