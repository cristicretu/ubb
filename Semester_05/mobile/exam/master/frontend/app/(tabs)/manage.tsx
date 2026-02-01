import React, { useState, useEffect, useCallback } from 'react';
import {
  View,
  StyleSheet,
  FlatList,
  TouchableOpacity,
  Alert,
  RefreshControl,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getAllDocuments, deleteDocument } from '@/utils/api';
import { Document } from '@/types/document';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import { useWebSocket } from '@/hooks/useWebSocket';

export default function ManageSection() {
  const [documents, setDocuments] = useState<Document[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isOnline, setIsOnline] = useState(true);
  const [refreshing, setRefreshing] = useState(false);
  const [deletingId, setDeletingId] = useState<number | null>(null);

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
    if (isOnline) {
      loadDocuments();
    }
  }, [isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name && message.size && message.owner) {
        Alert.alert(
          'New Document',
          `Name: ${message.name}\nSize: ${message.size}KB\nOwner: ${message.owner}`
        );
        loadDocuments();
      }
    },
  });

  const loadDocuments = async () => {
    if (!isOnline) {
      Alert.alert('Offline', 'This section requires internet connection.');
      return;
    }

    setIsLoading(true);
    log('Loading documents...', 'info');

    try {
      const response = await getAllDocuments();
      if (response.error) {
        log(`Error: ${response.error}`, 'error');
        Alert.alert('Error', response.error);
      } else if (response.data) {
        setDocuments(response.data);
        log(`Loaded ${response.data.length} documents`, 'success');
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
    if (!isOnline) {
      Alert.alert('Offline', 'This section requires internet connection.');
      return;
    }
    setRefreshing(true);
    await loadDocuments();
    setRefreshing(false);
  }, [isOnline]);

  const handleDelete = (document: Document) => {
    if (!isOnline) {
      Alert.alert('Offline', 'This section requires internet connection.');
      return;
    }

    Alert.alert(
      'Delete Document',
      `Are you sure you want to delete "${document.name}"?`,
      [
        { text: 'Cancel', style: 'cancel' },
        {
          text: 'Delete',
          style: 'destructive',
          onPress: () => performDelete(document.id),
        },
      ]
    );
  };

  const performDelete = async (id: number) => {
    if (!isOnline) {
      Alert.alert('Offline', 'This section requires internet connection.');
      return;
    }

    setDeletingId(id);
    log(`Deleting document: ${id}`, 'info');

    try {
      const response = await deleteDocument(id);
      if (response.error) {
        log(`Error: ${response.error}`, 'error');
        Alert.alert('Error', response.error);
      } else {
        log(`Deleted document: ${id}`, 'success');
        Alert.alert('Success', 'Document deleted successfully');
        await loadDocuments();
      }
    } catch (error) {
      const msg = error instanceof Error ? error.message : 'Unknown error';
      log(`Error: ${msg}`, 'error');
      Alert.alert('Error', msg);
    } finally {
      setDeletingId(null);
    }
  };

  const renderDocumentItem = ({ item }: { item: Document }) => (
    <ThemedView style={styles.documentItem}>
      <View style={styles.documentContent}>
        <ThemedText type="defaultSemiBold" style={styles.documentName}>
          {item.name}
        </ThemedText>
        <ThemedText style={styles.documentDetail}>Size: {item.size}KB</ThemedText>
        <ThemedText style={styles.documentDetail}>Usage: {item.usage}</ThemedText>
      </View>
      <TouchableOpacity
        style={[
          styles.deleteButton,
          deletingId === item.id && styles.deleteButtonDisabled,
        ]}
        onPress={() => handleDelete(item)}
        disabled={deletingId === item.id || !isOnline}>
        {deletingId === item.id ? (
          <ActivityIndicator size="small" color="#fff" />
        ) : (
          <ThemedText style={styles.deleteButtonText}>Delete</ThemedText>
        )}
      </TouchableOpacity>
    </ThemedView>
  );

  if (!isOnline) {
    return (
      <ThemedView style={styles.container}>
        <ThemedView style={styles.offlineContainer}>
          <ThemedText type="title" style={styles.offlineTitle}>
            Offline
          </ThemedText>
          <ThemedText style={styles.offlineMessage}>
            This section requires internet connection.
          </ThemedText>
        </ThemedView>
      </ThemedView>
    );
  }

  return (
    <ThemedView style={styles.container}>
      {isLoading && !refreshing ? (
        <LoadingSpinner visible={true} message="Loading documents..." />
      ) : (
        <FlatList
          data={documents}
          renderItem={renderDocumentItem}
          keyExtractor={(item) => item.id.toString()}
          contentContainerStyle={styles.listContent}
          ListEmptyComponent={
            <ThemedView style={styles.emptyContainer}>
              <ThemedText style={styles.emptyText}>No documents found</ThemedText>
            </ThemedView>
          }
          refreshControl={
            <RefreshControl refreshing={refreshing} onRefresh={onRefresh} />
          }
        />
      )}
    </ThemedView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  offlineContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 32,
  },
  offlineTitle: {
    marginBottom: 16,
    textAlign: 'center',
  },
  offlineMessage: {
    textAlign: 'center',
    fontSize: 16,
    color: '#666',
  },
  listContent: {
    padding: 16,
  },
  documentItem: {
    flexDirection: 'row',
    padding: 16,
    marginBottom: 12,
    borderRadius: 8,
    borderWidth: 1,
    borderColor: '#ddd',
    backgroundColor: '#f9f9f9',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  documentContent: {
    flex: 1,
    marginRight: 12,
  },
  documentName: {
    fontSize: 18,
    marginBottom: 8,
  },
  documentDetail: {
    fontSize: 14,
    marginBottom: 4,
    color: '#666',
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
});
