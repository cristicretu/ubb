import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  FlatList,
  Alert,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getAllDocuments } from '@/utils/api';
import { getLocalDocs, saveDocs } from '@/utils/storage';
import { Document } from '@/types/document';
import LoadingSpinner from '@/components/LoadingSpinner';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import { useWebSocket } from '@/hooks/useWebSocket';
import { log } from '@/utils/logger';

export default function StatusSection() {
  const [documents, setDocuments] = useState<Document[]>([]);
  const [loading, setLoading] = useState(false);
  const [isOnline, setIsOnline] = useState(false);

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
    loadDocuments();
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
    // Try to load from cache first
    const cachedDocs = await getLocalDocs();
    if (cachedDocs.length > 0) {
      const top10 = getTop10ByUsage(cachedDocs);
      setDocuments(top10);
    }

    if (!isOnline) {
      return;
    }

    setLoading(true);
    const response = await getAllDocuments();
    setLoading(false);

    if (response.error) {
      log(`Error loading documents: ${response.error}`, 'error');
      return;
    }

    if (response.data) {
      // Cache the data after fetch
      await saveDocs(response.data);
      
      // Get top 10 by usage
      const top10 = getTop10ByUsage(response.data);
      setDocuments(top10);
    }
  };

  const getTop10ByUsage = (docs: Document[]): Document[] => {
    return [...docs]
      .sort((a, b) => b.usage - a.usage)
      .slice(0, 10);
  };

  const renderDocumentItem = ({ item }: { item: Document }) => (
    <ThemedView style={styles.documentItem}>
      <ThemedText type="defaultSemiBold" style={styles.documentName}>
        {item.name}
      </ThemedText>
      <ThemedText style={styles.documentInfo}>
        Status: {item.status} | Usage: {item.usage} | Owner: {item.owner}
      </ThemedText>
    </ThemedView>
  );

  return (
    <ThemedView style={styles.container}>
      {loading ? (
        <LoadingSpinner visible={true} message="Loading documents..." />
      ) : (
        <>
          {!isOnline && (
            <View style={styles.offlineContainer}>
              <ThemedText style={styles.offlineMessage}>
                Offline - Showing cached data
              </ThemedText>
            </View>
          )}
          {documents.length === 0 ? (
            <View style={styles.emptyContainer}>
              <ThemedText style={styles.emptyMessage}>No documents found</ThemedText>
            </View>
          ) : (
            <FlatList
              data={documents}
              renderItem={renderDocumentItem}
              keyExtractor={(item) => item.id.toString()}
              contentContainerStyle={styles.listContent}
              ItemSeparatorComponent={() => <View style={styles.separator} />}
            />
          )}
        </>
      )}
    </ThemedView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  offlineContainer: {
    padding: 16,
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
    margin: 16,
    alignItems: 'center',
  },
  offlineMessage: {
    fontSize: 14,
    color: '#856404',
    textAlign: 'center',
  },
  listContent: {
    padding: 16,
  },
  documentItem: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  documentName: {
    fontSize: 16,
    marginBottom: 4,
  },
  documentInfo: {
    fontSize: 14,
    color: '#666',
  },
  separator: {
    height: 8,
  },
  emptyContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    padding: 40,
  },
  emptyMessage: {
    fontSize: 16,
    color: '#999',
    textAlign: 'center',
  },
});
