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
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import LoadingSpinner from '@/components/LoadingSpinner';
import { getAllFiles, getLocations, getFilesByLocation, deleteFile } from '@/utils/api';
import { getLocalFiles, saveFiles } from '@/utils/storage';
import { log } from '@/utils/logger';
import { FileItem } from '@/types/file';
import { useWebSocket } from '@/hooks/useWebSocket';

export default function ManageScreen() {
  const [isOnline, setIsOnline] = useState(false);
  const [locations, setLocations] = useState<string[]>([]);
  const [selectedLocation, setSelectedLocation] = useState<string | null>(null);
  const [locationFiles, setLocationFiles] = useState<FileItem[]>([]);
  const [topFiles, setTopFiles] = useState<FileItem[]>([]);
  const [loadingLocations, setLoadingLocations] = useState(false);
  const [loadingFiles, setLoadingFiles] = useState(false);
  const [loadingTopFiles, setLoadingTopFiles] = useState(false);
  const [deletingFileId, setDeletingFileId] = useState<number | null>(null);

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
    if (isOnline) fetchLocations();
  }, [isOnline]);

  useEffect(() => {
    if (isOnline) fetchTopFiles();
  }, [isOnline]);

  useEffect(() => {
    if (selectedLocation && isOnline) fetchFilesForLocation(selectedLocation);
    else if (selectedLocation && !isOnline) setLocationFiles([]);
  }, [selectedLocation, isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name && message.size && message.location) {
        Alert.alert(
          'New File',
          `Name: ${message.name}\nSize: ${message.size}KB\nLocation: ${message.location}`
        );
        if (isOnline) {
          fetchLocations();
          fetchTopFiles();
          if (selectedLocation) fetchFilesForLocation(selectedLocation);
        }
      }
    },
  });

  const showToast = (title: string, message: string, type: 'success' | 'error') => {
    log(`${title}: ${message}`, type);
    Alert.alert(title, message);
  };

  const fetchLocations = async () => {
    if (!isOnline) return showToast('Offline', 'Requires internet', 'error');
    setLoadingLocations(true);
    log('Fetching locations', 'info');
    
    const response = await getLocations();
    if (response.error) {
      showToast('Error', response.error, 'error');
    } else if (response.data) {
      setLocations(response.data);
      log(`Fetched ${response.data.length} locations`, 'success');
    }
    setLoadingLocations(false);
  };

  const fetchFilesForLocation = async (location: string) => {
    if (!isOnline) return showToast('Offline', 'Requires internet', 'error');
    setLoadingFiles(true);
    log(`Fetching files for: ${location}`, 'info');
    
    const response = await getFilesByLocation(location);
    if (response.error) {
      showToast('Error', response.error, 'error');
    } else if (response.data) {
      setLocationFiles(response.data);
      log(`Fetched ${response.data.length} files`, 'success');
    }
    setLoadingFiles(false);
  };

  const fetchTopFiles = async () => {
    if (!isOnline) return showToast('Offline', 'Requires internet', 'error');
    setLoadingTopFiles(true);
    log('Fetching top files', 'info');
    
    let allFiles = await getLocalFiles();
    if (allFiles.length === 0) {
      const response = await getAllFiles();
      if (response.error) {
        showToast('Error', response.error, 'error');
        setLoadingTopFiles(false);
        return;
      }
      if (response.data) {
        allFiles = response.data;
        await saveFiles(response.data);
        log(`Fetched ${response.data.length} files`, 'success');
      }
    }

    const sorted = [...allFiles].sort((a, b) => b.usage - a.usage);
    setTopFiles(sorted.slice(0, 10));
    log(`Top ${Math.min(10, sorted.length)} files by usage`, 'success');
    setLoadingTopFiles(false);
  };

  const handleDeleteFile = (file: FileItem) => {
    if (!isOnline) return showToast('Offline', 'Requires internet', 'error');
    Alert.alert('Delete File', `Delete "${file.name}"?`, [
      { text: 'Cancel', style: 'cancel' },
      { text: 'Delete', style: 'destructive', onPress: () => performDelete(file.id) },
    ]);
  };

  const performDelete = async (fileId: number) => {
    if (!isOnline) return showToast('Offline', 'Requires internet', 'error');
    setDeletingFileId(fileId);
    log(`Deleting file: ${fileId}`, 'info');
    
    const response = await deleteFile(fileId);
    if (response.error) {
      showToast('Error', response.error, 'error');
    } else {
      log(`Deleted file: ${fileId}`, 'success');
      showToast('Success', 'File deleted', 'success');
      if (selectedLocation) await fetchFilesForLocation(selectedLocation);
      await fetchTopFiles();
    }
    setDeletingFileId(null);
  };

  const renderLocationChip = ({ item }: { item: string }) => (
    <TouchableOpacity
      style={[styles.locationChip, selectedLocation === item && styles.locationChipSelected]}
      onPress={() => { log(`Selected: ${item}`, 'info'); setSelectedLocation(item); }}>
      <ThemedText style={[styles.locationChipText, selectedLocation === item && styles.locationChipTextSelected]}>{item}</ThemedText>
    </TouchableOpacity>
  );

  const renderFileItem = ({ item }: { item: FileItem }) => (
    <ThemedView style={styles.fileItem}>
      <View style={styles.fileItemContent}>
        <ThemedText type="defaultSemiBold" style={styles.fileName}>{item.name}</ThemedText>
        <ThemedText style={styles.fileInfo}>Status: {item.status} | Size: {item.size}KB | Usage: {item.usage}</ThemedText>
        {item.location && <ThemedText style={styles.fileLocation}>Location: {item.location}</ThemedText>}
      </View>
      <TouchableOpacity
        style={[styles.deleteButton, deletingFileId === item.id && styles.deleteButtonDisabled]}
        onPress={() => handleDeleteFile(item)}
        disabled={deletingFileId === item.id || !isOnline}>
        {deletingFileId === item.id ? <ActivityIndicator size="small" color="#fff" /> : <ThemedText style={styles.deleteButtonText}>Delete</ThemedText>}
      </TouchableOpacity>
    </ThemedView>
  );

  const renderTopFileItem = ({ item }: { item: FileItem }) => (
    <ThemedView style={styles.topFileItem}>
      <ThemedText type="defaultSemiBold" style={styles.topFileName}>{item.name}</ThemedText>
      <ThemedText style={styles.topFileInfo}>Status: {item.status} | Usage: {item.usage}</ThemedText>
      {item.location && <ThemedText style={styles.topFileLocation}>Location: {item.location}</ThemedText>}
    </ThemedView>
  );

  if (!isOnline) {
    return (
      <ScrollView style={styles.container} contentContainerStyle={styles.contentContainer}>
        <ThemedView style={styles.offlineContainer}>
          <ThemedText type="title" style={styles.offlineTitle}>Online Only</ThemedText>
          <ThemedText style={styles.offlineMessage}>This section requires internet connection.</ThemedText>
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
          refreshing={loadingLocations || loadingFiles || loadingTopFiles}
          onRefresh={() => {
            if (isOnline) {
              fetchLocations();
              fetchTopFiles();
              if (selectedLocation) fetchFilesForLocation(selectedLocation);
            }
          }}
        />
      }>

      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>Locations</ThemedText>
        {loadingLocations ? (
          <LoadingSpinner visible={true} message="Loading locations..." />
        ) : locations.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>No locations</ThemedText>
        ) : (
          <FlatList data={locations} renderItem={renderLocationChip} keyExtractor={(item) => item} horizontal showsHorizontalScrollIndicator={false} contentContainerStyle={styles.locationsList} />
        )}
      </ThemedView>

      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>Files in Location</ThemedText>
        {!selectedLocation ? (
          <ThemedText style={styles.emptyMessage}>Select a location above</ThemedText>
        ) : loadingFiles ? (
          <LoadingSpinner visible={true} message={`Loading ${selectedLocation}...`} />
        ) : locationFiles.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>No files in {selectedLocation}</ThemedText>
        ) : (
          <FlatList data={locationFiles} renderItem={renderFileItem} keyExtractor={(item) => item.id.toString()} scrollEnabled={false} ItemSeparatorComponent={() => <View style={styles.separator} />} />
        )}
      </ThemedView>

      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>Top 10 by Usage</ThemedText>
        {loadingTopFiles ? (
          <LoadingSpinner visible={true} message="Loading top files..." />
        ) : topFiles.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>No files</ThemedText>
        ) : (
          <FlatList data={topFiles} renderItem={renderTopFileItem} keyExtractor={(item) => item.id.toString()} scrollEnabled={false} ItemSeparatorComponent={() => <View style={styles.separator} />} />
        )}
      </ThemedView>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  container: { flex: 1 },
  contentContainer: { padding: 16 },
  offlineContainer: { flex: 1, justifyContent: 'center', alignItems: 'center', padding: 32, minHeight: 400 },
  offlineTitle: { marginBottom: 16, textAlign: 'center' },
  offlineMessage: { textAlign: 'center', fontSize: 16 },
  section: { marginBottom: 32 },
  sectionTitle: { marginBottom: 16 },
  locationsList: { paddingVertical: 8 },
  locationChip: { paddingHorizontal: 16, paddingVertical: 8, borderRadius: 20, backgroundColor: '#E5E5E5', marginRight: 8, borderWidth: 2, borderColor: 'transparent' },
  locationChipSelected: { backgroundColor: '#007AFF', borderColor: '#0051D5' },
  locationChipText: { fontSize: 14, color: '#333' },
  locationChipTextSelected: { color: '#FFF', fontWeight: '600' },
  emptyMessage: { fontSize: 14, color: '#666', fontStyle: 'italic', textAlign: 'center', padding: 16 },
  fileItem: { flexDirection: 'row', padding: 16, borderRadius: 8, backgroundColor: '#F5F5F5', alignItems: 'center', justifyContent: 'space-between' },
  fileItemContent: { flex: 1, marginRight: 12 },
  fileName: { fontSize: 16, marginBottom: 4 },
  fileInfo: { fontSize: 14, color: '#666', marginBottom: 2 },
  fileLocation: { fontSize: 12, color: '#999', marginTop: 4 },
  deleteButton: { backgroundColor: '#FF3B30', paddingHorizontal: 16, paddingVertical: 8, borderRadius: 6, minWidth: 70, alignItems: 'center', justifyContent: 'center' },
  deleteButtonDisabled: { opacity: 0.6 },
  deleteButtonText: { color: '#FFF', fontSize: 14, fontWeight: '600' },
  topFileItem: { padding: 16, borderRadius: 8, backgroundColor: '#F5F5F5' },
  topFileName: { fontSize: 16, marginBottom: 4 },
  topFileInfo: { fontSize: 14, color: '#666', marginBottom: 2 },
  topFileLocation: { fontSize: 12, color: '#999', marginTop: 4 },
  separator: { height: 8 },
});
