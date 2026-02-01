import React, { useState, useEffect } from 'react';
import {
  View,
  TextInput,
  StyleSheet,
  ScrollView,
  FlatList,
  TouchableOpacity,
  Alert,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getMyCabs } from '@/utils/api';
import { saveDriverName, getDriverName } from '@/utils/storage';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { Cab } from '@/types/cab';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';

export default function DriverSection() {
  const [driverName, setDriverName] = useState('');
  const [savedDriverName, setSavedDriverName] = useState<string | null>(null);
  const [cabs, setCabs] = useState<Cab[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
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
    loadSavedDriverName();
  }, []);

  useEffect(() => {
    if (savedDriverName) {
      fetchDriverCabs();
    } else {
      setCabs([]);
    }
  }, [savedDriverName]);

  const loadSavedDriverName = async () => {
    try {
      const name = await getDriverName();
      if (name) {
        setSavedDriverName(name);
        setDriverName(name);
        log(`Loaded saved driver name: ${name}`, 'success');
      }
    } catch (error) {
      const msg = error instanceof Error ? error.message : 'Unknown error';
      log(`Error loading driver name: ${msg}`, 'error');
    }
  };

  const handleSaveDriverName = async () => {
    const trimmedName = driverName.trim();
    if (!trimmedName) {
      Alert.alert('Error', 'Please enter a driver name');
      return;
    }

    setIsSaving(true);
    log(`Saving driver name: ${trimmedName}`, 'info');

    try {
      await saveDriverName(trimmedName);
      setSavedDriverName(trimmedName);
      Alert.alert('Success', 'Driver name saved');
      log(`Saved driver name: ${trimmedName}`, 'success');
    } catch (error) {
      const msg = error instanceof Error ? error.message : 'Unknown error';
      log(`Error saving driver name: ${msg}`, 'error');
      Alert.alert('Error', msg);
    } finally {
      setIsSaving(false);
    }
  };

  const fetchDriverCabs = async () => {
    if (!savedDriverName) return;
    if (!isOnline) {
      Alert.alert('Offline', 'Internet connection required to fetch cabs');
      return;
    }

    setIsLoading(true);
    log(`Fetching cabs for driver: ${savedDriverName}`, 'info');

    try {
      const response = await getMyCabs(savedDriverName);
      if (response.error) {
        log(`Error: ${response.error}`, 'error');
        Alert.alert('Error', response.error);
        setCabs([]);
      } else if (response.data) {
        setCabs(response.data);
        log(`Loaded ${response.data.length} cabs for ${savedDriverName}`, 'success');
      }
    } catch (error) {
      const msg = error instanceof Error ? error.message : 'Unknown error';
      log(`Error: ${msg}`, 'error');
      Alert.alert('Error', msg);
      setCabs([]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleCabPress = (cab: Cab) => {
    Alert.alert(
      'Cab Details',
      `ID: ${cab.id}\n` +
      `Name: ${cab.name}\n` +
      `Status: ${cab.status}\n` +
      `Size: ${cab.size}\n` +
      `Driver: ${cab.driver}\n` +
      `Color: ${cab.color}\n` +
      `Capacity: ${cab.capacity}`,
      [{ text: 'OK' }]
    );
    log(`Showing details for cab: ${cab.name}`, 'info');
  };

  const renderCabItem = ({ item }: { item: Cab }) => (
    <TouchableOpacity onPress={() => handleCabPress(item)}>
      <ThemedView style={styles.cabItem}>
        <ThemedText type="defaultSemiBold" style={styles.cabName}>{item.name}</ThemedText>
        <ThemedText style={styles.cabDetail}>Status: {item.status}</ThemedText>
        <ThemedText style={styles.cabDetail}>Size: {item.size}</ThemedText>
      </ThemedView>
    </TouchableOpacity>
  );

  return (
    <ThemedView style={styles.container}>
      <ScrollView style={styles.scrollView} contentContainerStyle={styles.scrollContent}>
        <ThemedView style={styles.section}>
          <ThemedText type="title" style={styles.sectionTitle}>Driver Settings</ThemedText>

          <ThemedText style={styles.label}>Driver Name</ThemedText>
          <TextInput
            style={styles.input}
            value={driverName}
            onChangeText={setDriverName}
            placeholder="Enter driver name"
            placeholderTextColor="#999"
            autoCapitalize="none"
            editable={!isSaving}
          />

          <TouchableOpacity
            style={[styles.saveButton, isSaving && styles.saveButtonDisabled]}
            onPress={handleSaveDriverName}
            disabled={isSaving}>
            {isSaving ? (
              <View style={styles.saveButtonLoading}>
                <ActivityIndicator size="small" color="#fff" />
                <ThemedText style={styles.saveButtonText}>Saving...</ThemedText>
              </View>
            ) : (
              <ThemedText style={styles.saveButtonText}>Save Driver Name</ThemedText>
            )}
          </TouchableOpacity>

          {savedDriverName && (
            <ThemedText style={styles.savedIndicator}>
              Saved driver: {savedDriverName}
            </ThemedText>
          )}
        </ThemedView>

        {savedDriverName && (
          <ThemedView style={styles.section}>
            <ThemedText type="title" style={styles.sectionTitle}>My Cabs</ThemedText>

            {isLoading ? (
              <LoadingSpinner visible={true} message="Loading cabs..." />
            ) : !isOnline ? (
              <ThemedView style={styles.offlineContainer}>
                <ThemedText style={styles.offlineMessage}>
                  You are offline. Internet connection required to fetch cabs.
                </ThemedText>
              </ThemedView>
            ) : cabs.length === 0 ? (
              <ThemedView style={styles.emptyContainer}>
                <ThemedText style={styles.emptyText}>No cabs found for {savedDriverName}</ThemedText>
              </ThemedView>
            ) : (
              <FlatList
                data={cabs}
                renderItem={renderCabItem}
                keyExtractor={(item) => item.id.toString()}
                scrollEnabled={false}
                contentContainerStyle={styles.listContent}
              />
            )}
          </ThemedView>
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
    marginBottom: 24,
  },
  sectionTitle: {
    marginBottom: 16,
    fontSize: 24,
  },
  label: {
    fontSize: 14,
    fontWeight: '600',
    marginBottom: 8,
    marginTop: 12,
  },
  input: {
    borderWidth: 1,
    borderColor: '#ddd',
    borderRadius: 8,
    padding: 12,
    fontSize: 16,
    backgroundColor: '#fff',
    color: '#000',
  },
  saveButton: {
    backgroundColor: '#007AFF',
    borderRadius: 8,
    padding: 16,
    alignItems: 'center',
    justifyContent: 'center',
    marginTop: 20,
    minHeight: 50,
  },
  saveButtonDisabled: {
    opacity: 0.6,
  },
  saveButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
  saveButtonLoading: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  savedIndicator: {
    marginTop: 12,
    fontSize: 14,
    color: '#007AFF',
    textAlign: 'center',
  },
  offlineContainer: {
    padding: 20,
    alignItems: 'center',
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#FFE69C',
  },
  offlineMessage: {
    fontSize: 16,
    textAlign: 'center',
    color: '#856404',
  },
  emptyContainer: {
    padding: 40,
    alignItems: 'center',
  },
  emptyText: {
    fontSize: 16,
    color: '#999',
  },
  listContent: {
    paddingBottom: 16,
  },
  cabItem: {
    padding: 16,
    marginBottom: 12,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#ddd',
    backgroundColor: '#f9f9f9',
  },
  cabName: {
    fontSize: 18,
    marginBottom: 8,
  },
  cabDetail: {
    fontSize: 14,
    marginBottom: 4,
    color: '#666',
  },
});
