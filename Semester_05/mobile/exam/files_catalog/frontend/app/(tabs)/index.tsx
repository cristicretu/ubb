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

  // WebSocket integration
  const handleWebSocketMessage = useCallback((message: any) => {
    log('WebSocket message received', 'info');
    
    // Check if message contains file data
    if (message && typeof message === 'object') {
      // If it's a file object with the expected structure
      if (message.name && message.size !== undefined && message.location) {
        const newFile: FileItem = {
          id: message.id || Date.now(),
          name: message.name,
          status: message.status || '',
          size: message.size,
          location: message.location,
          usage: message.usage || 0,
        };

        // Show alert with file details
        Alert.alert(
          'New File Added',
          `Name: ${newFile.name}\nSize: ${newFile.size}KB\nLocation: ${newFile.location}`,
          [{ text: 'OK' }]
        );

        // Add to the list
        setFiles((prevFiles) => {
          // Check if file already exists
          const exists = prevFiles.some((f) => f.id === newFile.id);
          if (exists) {
            return prevFiles;
          }
          return [newFile, ...prevFiles];
        });

        log(`New file added via WebSocket: ${newFile.name}`, 'success');
      }
    }
  }, []);

  const { isConnected: wsConnected } = useWebSocket({
    onMessage: handleWebSocketMessage,
  });

  // Check network status
  useEffect(() => {
    const unsubscribe = NetInfo.addEventListener((state) => {
      const online = state.isConnected ?? false;
      setIsOnline(online);
      log(`Network status changed: ${online ? 'online' : 'offline'}`, 'info');
    });

    // Check initial network status
    NetInfo.fetch().then((state) => {
      setIsOnline(state.isConnected ?? false);
    });

    return () => {
      unsubscribe();
    };
  }, []);

  // Load files on mount
  useEffect(() => {
    loadFiles();
  }, []);

  // Load files from API or cache
  const loadFiles = async () => {
    setIsLoading(true);
    log('Loading files...', 'info');

    try {
      // Try to load from API if online
      if (isOnline) {
        const response = await getAllFiles();
        if (response.error) {
          log(`Error fetching files: ${response.error}`, 'error');
          // Fall back to local storage
          const localFiles = await getLocalFiles();
          setFiles(localFiles);
          Alert.alert('Error', `Failed to fetch files: ${response.error}`);
        } else if (response.data) {
          setFiles(response.data);
          // Cache the files locally
          await saveFiles(response.data);
          log(`Loaded ${response.data.length} files from API`, 'success');
        }
      } else {
        // Load from local storage when offline
        const localFiles = await getLocalFiles();
        setFiles(localFiles);
        log(`Loaded ${localFiles.length} files from local storage`, 'info');
      }
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      log(`Error loading files: ${errorMessage}`, 'error');
      Alert.alert('Error', `Failed to load files: ${errorMessage}`);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle refresh
  const onRefresh = useCallback(async () => {
    setRefreshing(true);
    await loadFiles();
    setRefreshing(false);
  }, [isOnline]);

  // Handle form submission
  const handleSubmit = async () => {
    // Validation
    if (!name.trim()) {
      Alert.alert('Validation Error', 'Please enter a file name');
      return;
    }
    if (!status.trim()) {
      Alert.alert('Validation Error', 'Please enter a status');
      return;
    }
    if (!size.trim() || isNaN(Number(size))) {
      Alert.alert('Validation Error', 'Please enter a valid numeric size');
      return;
    }
    if (!location.trim()) {
      Alert.alert('Validation Error', 'Please enter a location');
      return;
    }

    setIsSubmitting(true);
    log(`Submitting file: ${name}`, 'info');

    const fileData = {
      name: name.trim(),
      status: status.trim(),
      size: Number(size),
      location: location.trim(),
    };

    try {
      if (isOnline) {
        // Try to create file via API
        const response = await createFile(fileData);
        if (response.error) {
          log(`Error creating file: ${response.error}`, 'error');
          Alert.alert('Error', `Failed to create file: ${response.error}`);
        } else if (response.data) {
          // Add to list
          setFiles((prevFiles) => [response.data!, ...prevFiles]);
          // Update cache
          await saveFiles([response.data, ...files]);
          Alert.alert('Success', 'File created successfully!');
          log(`File created successfully: ${response.data.name}`, 'success');
          // Reset form
          setName('');
          setStatus('');
          setSize('');
          setLocation('');
        }
      } else {
        // Save to pending files for later sync
        const pendingFile: FileItem = {
          id: Date.now(), // Temporary ID
          ...fileData,
          usage: 0,
        };
        await savePendingFile(pendingFile);
        // Add to local list
        setFiles((prevFiles) => [pendingFile, ...prevFiles]);
        Alert.alert(
          'Saved Offline',
          'File saved locally and will be synced when you go online.'
        );
        log(`File saved offline: ${pendingFile.name}`, 'success');
        // Reset form
        setName('');
        setStatus('');
        setSize('');
        setLocation('');
      }
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      log(`Error submitting file: ${errorMessage}`, 'error');
      Alert.alert('Error', `Failed to submit file: ${errorMessage}`);
    } finally {
      setIsSubmitting(false);
    }
  };

  // Handle retry when offline
  const handleRetry = async () => {
    if (isOnline) {
      await loadFiles();
    } else {
      Alert.alert('Offline', 'You are still offline. Please check your internet connection.');
    }
  };

  // Render file item
  const renderFileItem = ({ item }: { item: FileItem }) => (
    <ThemedView style={styles.fileItem}>
      <ThemedText type="defaultSemiBold" style={styles.fileName}>
        {item.name}
      </ThemedText>
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
        refreshControl={
          <RefreshControl refreshing={refreshing} onRefresh={onRefresh} />
        }>
        {/* Form Section */}
        <ThemedView style={styles.section}>
          <ThemedText type="title" style={styles.sectionTitle}>
            Record New File
          </ThemedText>

          <ThemedText style={styles.label}>Name</ThemedText>
          <TextInput
            style={styles.input}
            value={name}
            onChangeText={setName}
            placeholder="Enter file name"
            placeholderTextColor="#999"
            editable={!isSubmitting}
          />

          <ThemedText style={styles.label}>Status</ThemedText>
          <TextInput
            style={styles.input}
            value={status}
            onChangeText={setStatus}
            placeholder="e.g., shared, open, draft, secret"
            placeholderTextColor="#999"
            editable={!isSubmitting}
          />

          <ThemedText style={styles.label}>Size (KB)</ThemedText>
          <TextInput
            style={styles.input}
            value={size}
            onChangeText={setSize}
            placeholder="Enter file size in KB"
            placeholderTextColor="#999"
            keyboardType="numeric"
            editable={!isSubmitting}
          />

          <ThemedText style={styles.label}>Location</ThemedText>
          <TextInput
            style={styles.input}
            value={location}
            onChangeText={setLocation}
            placeholder="Enter file location"
            placeholderTextColor="#999"
            editable={!isSubmitting}
          />

          <TouchableOpacity
            style={[styles.submitButton, isSubmitting && styles.submitButtonDisabled]}
            onPress={handleSubmit}
            disabled={isSubmitting}>
            {isSubmitting ? (
              <View style={styles.submitButtonLoading}>
                <ActivityIndicator size="small" color="#fff" />
                <ThemedText style={styles.submitButtonText}>Submitting...</ThemedText>
              </View>
            ) : (
              <ThemedText style={styles.submitButtonText}>
                {isOnline ? 'Submit' : 'Save Offline'}
              </ThemedText>
            )}
          </TouchableOpacity>

          {!isOnline && (
            <ThemedText style={styles.offlineIndicator}>
              ⚠️ You are offline. File will be saved locally.
            </ThemedText>
          )}
        </ThemedView>

        {/* Files List Section */}
        <ThemedView style={styles.section}>
          <ThemedText type="title" style={styles.sectionTitle}>
            All Files
          </ThemedText>

          {isLoading ? (
            <LoadingSpinner visible={true} message="Loading files..." />
          ) : !isOnline && files.length === 0 ? (
            <ThemedView style={styles.offlineContainer}>
              <ThemedText style={styles.offlineMessage}>
                You are offline. No cached files available.
              </ThemedText>
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
                  <ThemedText style={styles.offlineBannerText}>
                    ⚠️ Showing cached files. You are offline.
                  </ThemedText>
                  <TouchableOpacity style={styles.retryButtonSmall} onPress={handleRetry}>
                    <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
                  </TouchableOpacity>
                </ThemedView>
              )}
              <FlatList
                data={files}
                renderItem={renderFileItem}
                keyExtractor={(item) => item.id.toString()}
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
  submitButton: {
    backgroundColor: '#007AFF',
    borderRadius: 8,
    padding: 16,
    alignItems: 'center',
    justifyContent: 'center',
    marginTop: 20,
    minHeight: 50,
  },
  submitButtonDisabled: {
    opacity: 0.6,
  },
  submitButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
  submitButtonLoading: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  offlineIndicator: {
    marginTop: 12,
    fontSize: 14,
    color: '#FF9500',
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
    marginBottom: 16,
    textAlign: 'center',
    color: '#856404',
  },
  offlineBanner: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: 12,
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
    marginBottom: 12,
    borderWidth: 1,
    borderColor: '#FFE69C',
  },
  offlineBannerText: {
    flex: 1,
    fontSize: 14,
    color: '#856404',
  },
  retryButton: {
    backgroundColor: '#007AFF',
    borderRadius: 8,
    paddingHorizontal: 24,
    paddingVertical: 12,
  },
  retryButtonSmall: {
    backgroundColor: '#007AFF',
    borderRadius: 6,
    paddingHorizontal: 16,
    paddingVertical: 8,
    marginLeft: 12,
  },
  retryButtonText: {
    color: '#fff',
    fontSize: 14,
    fontWeight: '600',
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
  fileItem: {
    padding: 16,
    marginBottom: 12,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#ddd',
    backgroundColor: '#f9f9f9',
  },
  fileName: {
    fontSize: 18,
    marginBottom: 8,
  },
  fileDetail: {
    fontSize: 14,
    marginBottom: 4,
    color: '#666',
  },
});
