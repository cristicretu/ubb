import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  FlatList,
  TouchableOpacity,
  Alert,
  RefreshControl,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getColors, getCabsByColor, deleteCab } from '@/utils/api';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { Cab } from '@/types/cab';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import { useWebSocket } from '@/hooks/useWebSocket';

export default function ManageSection() {
  const [isOnline, setIsOnline] = useState(false);
  const [colors, setColors] = useState<string[]>([]);
  const [selectedColor, setSelectedColor] = useState<string | null>(null);
  const [cabs, setCabs] = useState<Cab[]>([]);
  const [loadingColors, setLoadingColors] = useState(false);
  const [loadingCabs, setLoadingCabs] = useState(false);
  const [deletingCabId, setDeletingCabId] = useState<number | null>(null);

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
    if (isOnline) fetchColors();
  }, [isOnline]);

  useEffect(() => {
    if (selectedColor && isOnline) fetchCabsForColor(selectedColor);
    else if (selectedColor && !isOnline) setCabs([]);
  }, [selectedColor, isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name && message.size && message.driver) {
        Alert.alert(
          'New Cab',
          `Name: ${message.name}\nSize: ${message.size}\nDriver: ${message.driver}`
        );
        if (isOnline) {
          fetchColors();
          if (selectedColor) fetchCabsForColor(selectedColor);
        }
      }
    },
  });

  const showAlert = (title: string, message: string, type: 'success' | 'error') => {
    log(`${title}: ${message}`, type);
    Alert.alert(title, message);
  };

  const fetchColors = async () => {
    if (!isOnline) return showAlert('Offline', 'Requires internet connection', 'error');
    setLoadingColors(true);
    log('Fetching colors', 'info');
    
    const response = await getColors();
    if (response.error) {
      showAlert('Error', response.error, 'error');
    } else if (response.data) {
      setColors(response.data);
      log(`Fetched ${response.data.length} colors`, 'success');
    }
    setLoadingColors(false);
  };

  const fetchCabsForColor = async (color: string) => {
    if (!isOnline) return showAlert('Offline', 'Requires internet connection', 'error');
    setLoadingCabs(true);
    log(`Fetching cabs for color: ${color}`, 'info');
    
    const response = await getCabsByColor(color);
    if (response.error) {
      showAlert('Error', response.error, 'error');
    } else if (response.data) {
      setCabs(response.data);
      log(`Fetched ${response.data.length} cabs`, 'success');
    }
    setLoadingCabs(false);
  };

  const handleDeleteCab = (cab: Cab) => {
    if (!isOnline) return showAlert('Offline', 'Requires internet connection', 'error');
    Alert.alert(
      'Delete Cab',
      `Delete cab "${cab.name}" (ID: ${cab.id})?`,
      [
        { text: 'Cancel', style: 'cancel' },
        { text: 'Delete', style: 'destructive', onPress: () => performDelete(cab.id) },
      ]
    );
  };

  const performDelete = async (cabId: number) => {
    if (!isOnline) return showAlert('Offline', 'Requires internet connection', 'error');
    setDeletingCabId(cabId);
    log(`Deleting cab: ${cabId}`, 'info');
    
    const response = await deleteCab(cabId);
    if (response.error) {
      showAlert('Error', response.error, 'error');
    } else {
      log(`Deleted cab: ${cabId}`, 'success');
      showAlert('Success', 'Cab deleted successfully', 'success');
      if (selectedColor) await fetchCabsForColor(selectedColor);
    }
    setDeletingCabId(null);
  };

  const renderColorChip = ({ item }: { item: string }) => (
    <TouchableOpacity
      style={[styles.colorChip, selectedColor === item && styles.colorChipSelected]}
      onPress={() => {
        log(`Selected color: ${item}`, 'info');
        setSelectedColor(item);
      }}>
      <ThemedText
        style={[
          styles.colorChipText,
          selectedColor === item && styles.colorChipTextSelected,
        ]}>
        {item}
      </ThemedText>
    </TouchableOpacity>
  );

  const renderCabItem = ({ item }: { item: Cab }) => (
    <ThemedView style={styles.cabItem}>
      <View style={styles.cabItemContent}>
        <ThemedText type="defaultSemiBold" style={styles.cabName}>
          {item.name}
        </ThemedText>
        <ThemedText style={styles.cabInfo}>
          ID: {item.id} | Status: {item.status} | Size: {item.size}
        </ThemedText>
        <ThemedText style={styles.cabInfo}>
          Driver: {item.driver} | Capacity: {item.capacity}
        </ThemedText>
        <ThemedText style={styles.cabColor}>Color: {item.color}</ThemedText>
      </View>
      <TouchableOpacity
        style={[
          styles.deleteButton,
          deletingCabId === item.id && styles.deleteButtonDisabled,
        ]}
        onPress={() => handleDeleteCab(item)}
        disabled={deletingCabId === item.id || !isOnline}>
        {deletingCabId === item.id ? (
          <ActivityIndicator size="small" color="#fff" />
        ) : (
          <ThemedText style={styles.deleteButtonText}>Delete</ThemedText>
        )}
      </TouchableOpacity>
    </ThemedView>
  );

  if (!isOnline) {
    return (
      <ScrollView style={styles.container} contentContainerStyle={styles.contentContainer}>
        <ThemedView style={styles.offlineContainer}>
          <ThemedText type="title" style={styles.offlineTitle}>Online Only</ThemedText>
          <ThemedText style={styles.offlineMessage}>
            This section requires internet connection.
          </ThemedText>
        </ThemedView>
      </ScrollView>
    );
  }

  return (
    <ScrollView
      style={styles.container}
      contentContainerStyle={styles.contentContainer}
      refreshControl={
        <RefreshControl
          refreshing={loadingColors || loadingCabs}
          onRefresh={() => {
            if (isOnline) {
              fetchColors();
              if (selectedColor) fetchCabsForColor(selectedColor);
            }
          }}
        />
      }>
      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>Colors</ThemedText>
        {loadingColors ? (
          <LoadingSpinner visible={true} message="Loading colors..." />
        ) : colors.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>No colors available</ThemedText>
        ) : (
          <FlatList
            data={colors}
            renderItem={renderColorChip}
            keyExtractor={(item) => item}
            horizontal
            showsHorizontalScrollIndicator={false}
            contentContainerStyle={styles.colorsList}
          />
        )}
      </ThemedView>

      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>
          Cabs in Selected Color
        </ThemedText>
        {!selectedColor ? (
          <ThemedText style={styles.emptyMessage}>Select a color above</ThemedText>
        ) : loadingCabs ? (
          <LoadingSpinner visible={true} message={`Loading ${selectedColor} cabs...`} />
        ) : cabs.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>
            No cabs found for color {selectedColor}
          </ThemedText>
        ) : (
          <FlatList
            data={cabs}
            renderItem={renderCabItem}
            keyExtractor={(item) => item.id.toString()}
            scrollEnabled={false}
            ItemSeparatorComponent={() => <View style={styles.separator} />}
          />
        )}
      </ThemedView>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1 },
  contentContainer: { padding: 16 },
  offlineContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 32,
    minHeight: 400,
  },
  offlineTitle: { marginBottom: 16, textAlign: 'center' },
  offlineMessage: { textAlign: 'center', fontSize: 16 },
  section: { marginBottom: 32 },
  sectionTitle: { marginBottom: 16 },
  colorsList: { paddingVertical: 8 },
  colorChip: {
    paddingHorizontal: 16,
    paddingVertical: 8,
    borderRadius: 20,
    backgroundColor: '#E5E5E5',
    marginRight: 8,
    borderWidth: 2,
    borderColor: 'transparent',
  },
  colorChipSelected: {
    backgroundColor: '#007AFF',
    borderColor: '#0051D5',
  },
  colorChipText: { fontSize: 14, color: '#333' },
  colorChipTextSelected: { color: '#FFF', fontWeight: '600' },
  emptyMessage: {
    fontSize: 14,
    color: '#666',
    fontStyle: 'italic',
    textAlign: 'center',
    padding: 16,
  },
  cabItem: {
    flexDirection: 'row',
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  cabItemContent: { flex: 1, marginRight: 12 },
  cabName: { fontSize: 16, marginBottom: 4 },
  cabInfo: { fontSize: 14, color: '#666', marginBottom: 2 },
  cabColor: { fontSize: 12, color: '#999', marginTop: 4 },
  deleteButton: {
    backgroundColor: '#FF3B30',
    paddingHorizontal: 16,
    paddingVertical: 8,
    borderRadius: 6,
    minWidth: 70,
    alignItems: 'center',
    justifyContent: 'center',
  },
  deleteButtonDisabled: { opacity: 0.6 },
  deleteButtonText: { color: '#FFF', fontSize: 14, fontWeight: '600' },
  separator: { height: 8 },
});
