import React, { useState, useEffect, useCallback } from 'react';
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

export default function ManageScreen() {
  const [isOnline, setIsOnline] = useState<boolean>(false);
  const [locations, setLocations] = useState<string[]>([]);
  const [selectedLocation, setSelectedLocation] = useState<string | null>(null);
  const [locationFiles, setLocationFiles] = useState<FileItem[]>([]);
  const [topFiles, setTopFiles] = useState<FileItem[]>([]);
  
  const [loadingLocations, setLoadingLocations] = useState<boolean>(false);
  const [loadingFiles, setLoadingFiles] = useState<boolean>(false);
  const [loadingTopFiles, setLoadingTopFiles] = useState<boolean>(false);
  const [deletingFileId, setDeletingFileId] = useState<number | null>(null);

  // Check network connectivity
  useEffect(() => {
    const unsubscribe = NetInfo.addEventListener(state => {
      const online = state.isConnected ?? false;
      setIsOnline(online);
      log(`Network status: ${online ? 'Online' : 'Offline'}`, 'info');
      
      if (!online) {
        showToast('Offline', 'This feature requires an internet connection', 'error');
      }
    });

    // Check initial state
    NetInfo.fetch().then(state => {
      setIsOnline(state.isConnected ?? false);
    });

    return () => unsubscribe();
  }, []);

  // Fetch locations on mount
  useEffect(() => {
    if (isOnline) {
      fetchLocations();
    }
  }, [isOnline]);

  // Fetch top files on mount
  useEffect(() => {
    if (isOnline) {
      fetchTopFiles();
    }
  }, [isOnline]);

  // Fetch files when location is selected
  useEffect(() => {
    if (selectedLocation && isOnline) {
      fetchFilesForLocation(selectedLocation);
    } else if (selectedLocation && !isOnline) {
      setLocationFiles([]);
    }
  }, [selectedLocation, isOnline]);

  const showToast = (title: string, message: string, type: 'success' | 'error') => {
    log(`${title}: ${message}`, type);
    Alert.alert(title, message);
  };

  const fetchLocations = async () => {
    if (!isOnline) {
      showToast('Offline', 'This feature requires an internet connection', 'error');
      return;
    }

    setLoadingLocations(true);
    log('Fetching locations', 'info');
    
    const response = await getLocations();
    
    if (response.error) {
      log(`Error fetching locations: ${response.error}`, 'error');
      showToast('Error', `Failed to fetch locations: ${response.error}`, 'error');
      setLoadingLocations(false);
      return;
    }

    if (response.data) {
      setLocations(response.data);
      log(`Successfully fetched ${response.data.length} locations`, 'success');
    }
    
    setLoadingLocations(false);
  };

  const fetchFilesForLocation = async (location: string) => {
    if (!isOnline) {
      showToast('Offline', 'This feature requires an internet connection', 'error');
      return;
    }

    setLoadingFiles(true);
    log(`Fetching files for location: ${location}`, 'info');
    
    const response = await getFilesByLocation(location);
    
    if (response.error) {
      log(`Error fetching files: ${response.error}`, 'error');
      showToast('Error', `Failed to fetch files: ${response.error}`, 'error');
      setLoadingFiles(false);
      return;
    }

    if (response.data) {
      setLocationFiles(response.data);
      log(`Successfully fetched ${response.data.length} files for location ${location}`, 'success');
    }
    
    setLoadingFiles(false);
  };

  const fetchTopFiles = async () => {
    if (!isOnline) {
      showToast('Offline', 'This feature requires an internet connection', 'error');
      return;
    }

    setLoadingTopFiles(true);
    log('Fetching top files by usage', 'info');
    
    // Try to get cached files first
    let allFiles = await getLocalFiles();
    
    // If no cached files or want fresh data, fetch from API
    if (allFiles.length === 0) {
      const response = await getAllFiles();
      
      if (response.error) {
        log(`Error fetching all files: ${response.error}`, 'error');
        showToast('Error', `Failed to fetch files: ${response.error}`, 'error');
        setLoadingTopFiles(false);
        return;
      }

      if (response.data) {
        allFiles = response.data;
        // Cache the files
        await saveFiles(response.data);
        log(`Successfully fetched and cached ${response.data.length} files`, 'success');
      }
    } else {
      log(`Using cached files: ${allFiles.length} files`, 'info');
    }

    // Sort by usage descending and take top 10
    const sorted = [...allFiles].sort((a, b) => b.usage - a.usage);
    const top10 = sorted.slice(0, 10);
    setTopFiles(top10);
    log(`Displaying top ${top10.length} files by usage`, 'success');
    
    setLoadingTopFiles(false);
  };

  const handleDeleteFile = (file: FileItem) => {
    if (!isOnline) {
      showToast('Offline', 'This feature requires an internet connection', 'error');
      return;
    }

    Alert.alert(
      'Delete File',
      `Are you sure you want to delete "${file.name}"?`,
      [
        {
          text: 'Cancel',
          style: 'cancel',
        },
        {
          text: 'Delete',
          style: 'destructive',
          onPress: () => performDelete(file.id),
        },
      ]
    );
  };

  const performDelete = async (fileId: number) => {
    if (!isOnline) {
      showToast('Offline', 'This feature requires an internet connection', 'error');
      return;
    }

    setDeletingFileId(fileId);
    log(`Deleting file with ID: ${fileId}`, 'info');
    
    const response = await deleteFile(fileId);
    
    if (response.error) {
      log(`Error deleting file: ${response.error}`, 'error');
      showToast('Error', `Failed to delete file: ${response.error}`, 'error');
      setDeletingFileId(null);
      return;
    }

    log(`Successfully deleted file with ID: ${fileId}`, 'success');
    showToast('Success', 'File deleted successfully', 'success');
    
    // Refresh the files list
    if (selectedLocation) {
      await fetchFilesForLocation(selectedLocation);
    }
    
    // Refresh top files
    await fetchTopFiles();
    
    setDeletingFileId(null);
  };

  const formatFileSize = (bytes: number): string => {
    if (bytes === 0) return '0 B';
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return Math.round(bytes / Math.pow(k, i) * 100) / 100 + ' ' + sizes[i];
  };

  const renderLocationChip = ({ item }: { item: string }) => (
    <TouchableOpacity
      style={[
        styles.locationChip,
        selectedLocation === item && styles.locationChipSelected,
      ]}
      onPress={() => {
        log(`Location selected: ${item}`, 'info');
        setSelectedLocation(item);
      }}>
      <ThemedText
        style={[
          styles.locationChipText,
          selectedLocation === item && styles.locationChipTextSelected,
        ]}>
        {item}
      </ThemedText>
    </TouchableOpacity>
  );

  const renderFileItem = ({ item }: { item: FileItem }) => (
    <ThemedView style={styles.fileItem}>
      <View style={styles.fileItemContent}>
        <ThemedText type="defaultSemiBold" style={styles.fileName}>
          {item.name}
        </ThemedText>
        <ThemedText style={styles.fileInfo}>
          Status: {item.status} • Size: {formatFileSize(item.size)} • Usage: {item.usage}
        </ThemedText>
        {item.location && (
          <ThemedText style={styles.fileLocation}>Location: {item.location}</ThemedText>
        )}
      </View>
      <TouchableOpacity
        style={[
          styles.deleteButton,
          deletingFileId === item.id && styles.deleteButtonDisabled,
        ]}
        onPress={() => handleDeleteFile(item)}
        disabled={deletingFileId === item.id || !isOnline}>
        {deletingFileId === item.id ? (
          <ActivityIndicator size="small" color="#fff" />
        ) : (
          <ThemedText style={styles.deleteButtonText}>Delete</ThemedText>
        )}
      </TouchableOpacity>
    </ThemedView>
  );

  const renderTopFileItem = ({ item }: { item: FileItem }) => (
    <ThemedView style={styles.topFileItem}>
      <View style={styles.topFileContent}>
        <ThemedText type="defaultSemiBold" style={styles.topFileName}>
          {item.name}
        </ThemedText>
        <ThemedText style={styles.topFileInfo}>
          Status: {item.status} • Usage: {item.usage}
        </ThemedText>
        {item.location && (
          <ThemedText style={styles.topFileLocation}>Location: {item.location}</ThemedText>
        )}
      </View>
    </ThemedView>
  );

  if (!isOnline) {
    return (
      <ScrollView style={styles.container} contentContainerStyle={styles.contentContainer}>
        <ThemedView style={styles.offlineContainer}>
          <ThemedText type="title" style={styles.offlineTitle}>
            Online Only Feature
          </ThemedText>
          <ThemedText style={styles.offlineMessage}>
            This section requires an internet connection. Please check your network settings.
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
          refreshing={loadingLocations || loadingFiles || loadingTopFiles}
          onRefresh={() => {
            if (isOnline) {
              fetchLocations();
              fetchTopFiles();
              if (selectedLocation) {
                fetchFilesForLocation(selectedLocation);
              }
            }
          }}
        />
      }>
      {/* Section 1: Locations List */}
      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>
          Locations
        </ThemedText>
        {loadingLocations ? (
          <LoadingSpinner visible={true} message="Loading locations..." />
        ) : locations.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>No locations available</ThemedText>
        ) : (
          <FlatList
            data={locations}
            renderItem={renderLocationChip}
            keyExtractor={(item) => item}
            horizontal
            showsHorizontalScrollIndicator={false}
            contentContainerStyle={styles.locationsList}
          />
        )}
      </ThemedView>

      {/* Section 2: Files in Selected Location */}
      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>
          Files in Selected Location
        </ThemedText>
        {!selectedLocation ? (
          <ThemedText style={styles.emptyMessage}>
            Select a location above to view files
          </ThemedText>
        ) : loadingFiles ? (
          <LoadingSpinner visible={true} message={`Loading files for ${selectedLocation}...`} />
        ) : locationFiles.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>
            No files found in {selectedLocation}
          </ThemedText>
        ) : (
          <FlatList
            data={locationFiles}
            renderItem={renderFileItem}
            keyExtractor={(item) => item.id.toString()}
            scrollEnabled={false}
            ItemSeparatorComponent={() => <View style={styles.separator} />}
          />
        )}
      </ThemedView>

      {/* Section 3: Top 10 Files by Usage */}
      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>
          Top 10 Files by Usage
        </ThemedText>
        {loadingTopFiles ? (
          <LoadingSpinner visible={true} message="Loading top files..." />
        ) : topFiles.length === 0 ? (
          <ThemedText style={styles.emptyMessage}>No files available</ThemedText>
        ) : (
          <FlatList
            data={topFiles}
            renderItem={renderTopFileItem}
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
  container: {
    flex: 1,
  },
  contentContainer: {
    padding: 16,
  },
  offlineContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 32,
    minHeight: 400,
  },
  offlineTitle: {
    marginBottom: 16,
    textAlign: 'center',
  },
  offlineMessage: {
    textAlign: 'center',
    fontSize: 16,
  },
  section: {
    marginBottom: 32,
  },
  sectionTitle: {
    marginBottom: 16,
  },
  locationsList: {
    paddingVertical: 8,
  },
  locationChip: {
    paddingHorizontal: 16,
    paddingVertical: 8,
    borderRadius: 20,
    backgroundColor: '#E5E5E5',
    marginRight: 8,
    borderWidth: 2,
    borderColor: 'transparent',
  },
  locationChipSelected: {
    backgroundColor: '#007AFF',
    borderColor: '#0051D5',
  },
  locationChipText: {
    fontSize: 14,
    color: '#333',
  },
  locationChipTextSelected: {
    color: '#FFF',
    fontWeight: '600',
  },
  emptyMessage: {
    fontSize: 14,
    color: '#666',
    fontStyle: 'italic',
    textAlign: 'center',
    padding: 16,
  },
  fileItem: {
    flexDirection: 'row',
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  fileItemContent: {
    flex: 1,
    marginRight: 12,
  },
  fileName: {
    fontSize: 16,
    marginBottom: 4,
  },
  fileInfo: {
    fontSize: 14,
    color: '#666',
    marginBottom: 2,
  },
  fileLocation: {
    fontSize: 12,
    color: '#999',
    marginTop: 4,
  },
  deleteButton: {
    backgroundColor: '#FF3B30',
    paddingHorizontal: 16,
    paddingVertical: 8,
    borderRadius: 6,
    minWidth: 70,
    alignItems: 'center',
    justifyContent: 'center',
  },
  deleteButtonDisabled: {
    opacity: 0.6,
  },
  deleteButtonText: {
    color: '#FFF',
    fontSize: 14,
    fontWeight: '600',
  },
  topFileItem: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  topFileContent: {
    flex: 1,
  },
  topFileName: {
    fontSize: 16,
    marginBottom: 4,
  },
  topFileInfo: {
    fontSize: 14,
    color: '#666',
    marginBottom: 2,
  },
  topFileLocation: {
    fontSize: 12,
    color: '#999',
    marginTop: 4,
  },
  separator: {
    height: 8,
  },
});
