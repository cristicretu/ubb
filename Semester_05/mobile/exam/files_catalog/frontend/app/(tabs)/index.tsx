import React, { useState, useEffect, useCallback } from 'react';
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
import { createFile, getAllFiles } from '@/utils/api';
import { saveFiles, getLocalFiles, savePendingFile } from '@/utils/storage';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { useWebSocket } from '@/hooks/useWebSocket';
import { FileItem } from '@/types/file';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';

export default function RecordSection() {
  const [name, setName] = useState('');
  const [status, setStatus] = useState('');
  const [size, setSize] = useState('');
  const [location, setLocation] = useState('');
  const [files, setFiles] = useState<FileItem[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isOnline, setIsOnline] = useState(true);
  const [refreshing, setRefreshing] = useState(false);

  const handleWebSocketMessage = useCallback((message: any) => {
    log('WebSocket message received', 'info');
    
    if (message?.name && message?.size !== undefined && message?.location) {
      const newFile: FileItem = {
        id: message.id || Date.now(),
        name: message.name,
        status: message.status || '',
        size: message.size,
        location: message.location,
        usage: message.usage || 0,
      };

      Alert.alert(
        'New File Added',
        `Name: ${newFile.name}\nSize: ${newFile.size}KB\nLocation: ${newFile.location}`
      );

      setFiles((prev) => {
        if (prev.some((f) => f.id === newFile.id)) return prev;
        return [newFile, ...prev];
      });

      log(`New file via WebSocket: ${newFile.name}`, 'success');
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
    loadFiles();
  }, []);

  const loadFiles = async () => {
    setIsLoading(true);
    log('Loading files...', 'info');

    try {
      if (isOnline) {
        const response = await getAllFiles();
        if (response.error) {
          log(`Error: ${response.error}`, 'error');
          const localFiles = await getLocalFiles();
          setFiles(localFiles);
          Alert.alert('Error', response.error);
        } else if (response.data) {
          setFiles(response.data);
          await saveFiles(response.data);
          log(`Loaded ${response.data.length} files`, 'success');
        }
      } else {
        const localFiles = await getLocalFiles();
        setFiles(localFiles);
        log(`Loaded ${localFiles.length} cached files`, 'info');
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
    await loadFiles();
    setRefreshing(false);
  }, [isOnline]);

  const handleSubmit = async () => {
    if (!name.trim()) return Alert.alert('Error', 'Enter file name');
    if (!status.trim()) return Alert.alert('Error', 'Enter status');
    if (!size.trim() || isNaN(Number(size))) return Alert.alert('Error', 'Enter valid size');
    if (!location.trim()) return Alert.alert('Error', 'Enter location');

    setIsSubmitting(true);
    log(`Submitting: ${name}`, 'info');

    const fileData = {
      name: name.trim(),
      status: status.trim(),
      size: Number(size),
      location: location.trim(),
    };

    try {
      if (isOnline) {
        const response = await createFile(fileData);
        if (response.error) {
          log(`Error: ${response.error}`, 'error');
          Alert.alert('Error', response.error);
        } else if (response.data) {
          setFiles((prev) => [response.data!, ...prev]);
          await saveFiles([response.data, ...files]);
          Alert.alert('Success', 'File created!');
          log(`Created: ${response.data.name}`, 'success');
          setName(''); setStatus(''); setSize(''); setLocation('');
        }
      } else {
        const pendingFile: FileItem = { id: Date.now(), ...fileData, usage: 0 };
        await savePendingFile(pendingFile);
        setFiles((prev) => [pendingFile, ...prev]);
        Alert.alert('Saved Offline', 'Will sync when online.');
        log(`Saved offline: ${pendingFile.name}`, 'success');
        setName(''); setStatus(''); setSize(''); setLocation('');
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
    if (isOnline) {
      await loadFiles();
    } else {
      Alert.alert('Offline', 'Check your connection.');
    }
  };

  const renderFileItem = ({ item }: { item: FileItem }) => (
    <ThemedView style={styles.fileItem}>
      <ThemedText type="defaultSemiBold" style={styles.fileName}>{item.name}</ThemedText>
      <ThemedText style={styles.fileDetail}>ID: {item.id}</ThemedText>
      <ThemedText style={styles.fileDetail}>Status: {item.status}</ThemedText>
      <ThemedText style={styles.fileDetail}>Size: {item.size}KB</ThemedText>
      <ThemedText style={styles.fileDetail}>Location: {item.location}</ThemedText>
      <ThemedText style={styles.fileDetail}>Usage: {item.usage}</ThemedText>
    </ThemedView>
  );

  return (
    <ThemedView style={styles.container}>
      <ScrollView
        style={styles.scrollView}
        contentContainerStyle={styles.scrollContent}
        refreshControl={<RefreshControl refreshing={refreshing} onRefresh={onRefresh} />}>
        
        <ThemedView style={styles.section}>
          <ThemedText type="title" style={styles.sectionTitle}>Record New File</ThemedText>

          <ThemedText style={styles.label}>Name</ThemedText>
          <TextInput style={styles.input} value={name} onChangeText={setName} placeholder="File name" placeholderTextColor="#999" editable={!isSubmitting} />

          <ThemedText style={styles.label}>Status</ThemedText>
          <TextInput style={styles.input} value={status} onChangeText={setStatus} placeholder="shared, open, draft, secret" placeholderTextColor="#999" editable={!isSubmitting} />

          <ThemedText style={styles.label}>Size (KB)</ThemedText>
          <TextInput style={styles.input} value={size} onChangeText={setSize} placeholder="Size in KB" placeholderTextColor="#999" keyboardType="numeric" editable={!isSubmitting} />

          <ThemedText style={styles.label}>Location</ThemedText>
          <TextInput style={styles.input} value={location} onChangeText={setLocation} placeholder="File location" placeholderTextColor="#999" editable={!isSubmitting} />

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

          {!isOnline && <ThemedText style={styles.offlineIndicator}>You are offline. File will be saved locally.</ThemedText>}
        </ThemedView>

        <ThemedView style={styles.section}>
          <ThemedText type="title" style={styles.sectionTitle}>All Files</ThemedText>

          {isLoading ? (
            <LoadingSpinner visible={true} message="Loading files..." />
          ) : !isOnline && files.length === 0 ? (
            <ThemedView style={styles.offlineContainer}>
              <ThemedText style={styles.offlineMessage}>You are offline. No cached files.</ThemedText>
              <TouchableOpacity style={styles.retryButton} onPress={handleRetry}>
                <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
              </TouchableOpacity>
            </ThemedView>
          ) : files.length === 0 ? (
            <ThemedView style={styles.emptyContainer}>
              <ThemedText style={styles.emptyText}>No files found</ThemedText>
            </ThemedView>
          ) : (
            <>
              {!isOnline && (
                <ThemedView style={styles.offlineBanner}>
                  <ThemedText style={styles.offlineBannerText}>Showing cached files. You are offline.</ThemedText>
                  <TouchableOpacity style={styles.retryButtonSmall} onPress={handleRetry}>
                    <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
                  </TouchableOpacity>
                </ThemedView>
              )}
              <FlatList
                data={files}
                renderItem={renderFileItem}
                keyExtractor={(item, index) => `${item.id}-${index}`}
                scrollEnabled={false}
                contentContainerStyle={styles.listContent}
              />
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
  submitButton: { backgroundColor: '#007AFF', borderRadius: 8, padding: 16, alignItems: 'center', justifyContent: 'center', marginTop: 20, minHeight: 50 },
  submitButtonDisabled: { opacity: 0.6 },
  submitButtonText: { color: '#fff', fontSize: 16, fontWeight: '600' },
  submitButtonLoading: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  offlineIndicator: { marginTop: 12, fontSize: 14, color: '#FF9500', textAlign: 'center' },
  offlineContainer: { padding: 20, alignItems: 'center', backgroundColor: '#FFF3CD', borderRadius: 8, borderWidth: 1, borderColor: '#FFE69C' },
  offlineMessage: { fontSize: 16, marginBottom: 16, textAlign: 'center', color: '#856404' },
  offlineBanner: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', padding: 12, backgroundColor: '#FFF3CD', borderRadius: 8, marginBottom: 12, borderWidth: 1, borderColor: '#FFE69C' },
  offlineBannerText: { flex: 1, fontSize: 14, color: '#856404' },
  retryButton: { backgroundColor: '#007AFF', borderRadius: 8, paddingHorizontal: 24, paddingVertical: 12 },
  retryButtonSmall: { backgroundColor: '#007AFF', borderRadius: 6, paddingHorizontal: 16, paddingVertical: 8, marginLeft: 12 },
  retryButtonText: { color: '#fff', fontSize: 14, fontWeight: '600' },
  emptyContainer: { padding: 40, alignItems: 'center' },
  emptyText: { fontSize: 16, color: '#999' },
  listContent: { paddingBottom: 16 },
  fileItem: { padding: 16, marginBottom: 12, borderRadius: 8, borderWidth: 1, borderColor: '#ddd', backgroundColor: '#f9f9f9' },
  fileName: { fontSize: 18, marginBottom: 8 },
  fileDetail: { fontSize: 14, marginBottom: 4, color: '#666' },
});
