import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  TextInput,
  TouchableOpacity,
  Alert,
  FlatList,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import LoadingSpinner from '@/components/LoadingSpinner';
import { createDocument, getDocumentsByOwner } from '@/utils/api';
import { saveDocs, getLocalDocs, savePendingDoc, saveOwnerName, getOwnerName } from '@/utils/storage';
import { Document } from '@/types/document';
import { useWebSocket } from '@/hooks/useWebSocket';
import { log } from '@/utils/logger';

export default function OwnerSection() {
  const [isOnline, setIsOnline] = useState(false);
  const [ownerName, setOwnerName] = useState('');
  const [savedOwner, setSavedOwner] = useState<string | null>(null);
  const [documents, setDocuments] = useState<Document[]>([]);
  const [loading, setLoading] = useState(false);
  const [submitting, setSubmitting] = useState(false);
  
  // Document form fields
  const [docName, setDocName] = useState('');
  const [docStatus, setDocStatus] = useState('');
  const [docSize, setDocSize] = useState('');

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
    loadSavedOwner();
  }, []);

  useEffect(() => {
    if (savedOwner) {
      loadDocuments();
    }
  }, [savedOwner, isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name && message.size && message.owner) {
        Alert.alert(
          'New Document',
          `New Document: ${message.name}, Size: ${message.size}KB, Owner: ${message.owner}`
        );
        // Refresh documents if it's for the current owner
        if (savedOwner && message.owner.toLowerCase() === savedOwner.toLowerCase()) {
          loadDocuments();
        }
      }
    },
  });

  const loadSavedOwner = async () => {
    const owner = await getOwnerName();
    if (owner) {
      setSavedOwner(owner);
      setOwnerName(owner);
    }
  };

  const handleSaveOwner = async () => {
    const trimmed = ownerName.trim();
    if (!trimmed) {
      Alert.alert('Error', 'Owner name cannot be empty');
      return;
    }
    await saveOwnerName(trimmed);
    setSavedOwner(trimmed);
    Alert.alert('Success', 'Owner name saved');
  };

  const loadDocuments = async () => {
    if (!savedOwner) return;

    // Try to load from cache first
    const cachedDocs = await getLocalDocs();
    const ownerDocs = cachedDocs.filter(
      (doc) => doc.owner.toLowerCase() === savedOwner.toLowerCase()
    );
    
    if (ownerDocs.length > 0) {
      setDocuments(ownerDocs);
    }

    if (!isOnline) {
      return;
    }

    setLoading(true);
    const response = await getDocumentsByOwner(savedOwner);
    setLoading(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      setDocuments(response.data);
      // Cache all documents (merge with existing)
      const allCached = await getLocalDocs();
      const updatedDocs = [...allCached];
      response.data.forEach((doc) => {
        const index = updatedDocs.findIndex((d) => d.id === doc.id);
        if (index >= 0) {
          updatedDocs[index] = doc;
        } else {
          updatedDocs.push(doc);
        }
      });
      await saveDocs(updatedDocs);
    }
  };

  const handleSubmitDocument = async () => {
    if (!savedOwner) {
      Alert.alert('Error', 'Please save owner name first');
      return;
    }

    const trimmedName = docName.trim();
    const trimmedStatus = docStatus.trim();
    const sizeNum = parseInt(docSize);

    if (!trimmedName) {
      Alert.alert('Error', 'Document name is required');
      return;
    }
    if (!trimmedStatus) {
      Alert.alert('Error', 'Status is required');
      return;
    }
    if (isNaN(sizeNum) || sizeNum <= 0) {
      Alert.alert('Error', 'Size must be a positive number');
      return;
    }

    const newDoc: Omit<Document, 'id' | 'usage'> = {
      name: trimmedName,
      status: trimmedStatus,
      owner: savedOwner,
      size: sizeNum,
    };

    if (isOnline) {
      setSubmitting(true);
      const response = await createDocument(newDoc);
      setSubmitting(false);

      if (response.error) {
        Alert.alert('Error', response.error);
        // Save as pending on error
        await savePendingDoc({ ...newDoc, id: Date.now(), usage: 0 });
        return;
      }

      if (response.data) {
        Alert.alert('Success', 'Document created');
        setDocName('');
        setDocStatus('');
        setDocSize('');
        await loadDocuments();
      }
    } else {
      // Offline: save as pending
      await savePendingDoc({ ...newDoc, id: Date.now(), usage: 0 });
      Alert.alert('Offline', 'Document saved offline. It will be synced when online.');
      setDocName('');
      setDocStatus('');
      setDocSize('');
    }
  };

  const renderDocumentItem = ({ item }: { item: Document }) => (
    <ThemedView style={styles.documentItem}>
      <ThemedText type="defaultSemiBold" style={styles.documentName}>
        {item.name}
      </ThemedText>
      <ThemedText style={styles.documentInfo}>
        Status: {item.status} | Size: {item.size}KB | Usage: {item.usage}
      </ThemedText>
    </ThemedView>
  );

  return (
    <ScrollView style={styles.container} contentContainerStyle={styles.contentContainer}>
      {/* Owner Settings Section */}
      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>
          Owner Settings
        </ThemedText>
        <TextInput
          style={styles.input}
          placeholder="Enter owner name"
          value={ownerName}
          onChangeText={setOwnerName}
          autoCapitalize="none"
        />
        <TouchableOpacity style={styles.saveButton} onPress={handleSaveOwner}>
          <ThemedText style={styles.saveButtonText}>Save</ThemedText>
        </TouchableOpacity>
        {savedOwner && (
          <ThemedText style={styles.savedOwnerText}>Saved owner: {savedOwner}</ThemedText>
        )}
      </ThemedView>

      {/* Document Form Section - Only show if owner saved */}
      {savedOwner && (
        <ThemedView style={styles.section}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            Record Document
          </ThemedText>
          <TextInput
            style={styles.input}
            placeholder="Document name"
            value={docName}
            onChangeText={setDocName}
            autoCapitalize="none"
          />
          <TextInput
            style={styles.input}
            placeholder="Status (shared, open, draft, secret)"
            value={docStatus}
            onChangeText={setDocStatus}
            autoCapitalize="none"
          />
          <TextInput
            style={styles.input}
            placeholder="Size (KB)"
            value={docSize}
            onChangeText={setDocSize}
            keyboardType="numeric"
            autoCapitalize="none"
          />
          <ThemedText style={styles.ownerInfo}>Owner: {savedOwner}</ThemedText>
          <TouchableOpacity
            style={[styles.submitButton, submitting && styles.submitButtonDisabled]}
            onPress={handleSubmitDocument}
            disabled={submitting}>
            {submitting ? (
              <ActivityIndicator size="small" color="#fff" />
            ) : (
              <ThemedText style={styles.submitButtonText}>Submit</ThemedText>
            )}
          </TouchableOpacity>
        </ThemedView>
      )}

      {/* Documents List Section */}
      {savedOwner && (
        <ThemedView style={styles.section}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            My Documents
          </ThemedText>
          {!isOnline ? (
            <View style={styles.offlineContainer}>
              <ThemedText style={styles.offlineMessage}>Offline - Showing cached data</ThemedText>
              <TouchableOpacity style={styles.retryButton} onPress={loadDocuments}>
                <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
              </TouchableOpacity>
            </View>
          ) : null}
          {loading ? (
            <LoadingSpinner visible={true} message="Loading documents..." />
          ) : documents.length === 0 ? (
            <ThemedText style={styles.emptyMessage}>No documents found</ThemedText>
          ) : (
            <FlatList
              data={documents}
              renderItem={renderDocumentItem}
              keyExtractor={(item) => item.id.toString()}
              scrollEnabled={false}
              ItemSeparatorComponent={() => <View style={styles.separator} />}
            />
          )}
        </ThemedView>
      )}
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
  section: {
    marginBottom: 32,
  },
  sectionTitle: {
    marginBottom: 16,
  },
  input: {
    borderWidth: 1,
    borderColor: '#ccc',
    borderRadius: 8,
    padding: 12,
    fontSize: 16,
    marginBottom: 12,
    backgroundColor: '#fff',
  },
  saveButton: {
    backgroundColor: '#007AFF',
    paddingHorizontal: 24,
    paddingVertical: 12,
    borderRadius: 8,
    alignItems: 'center',
    marginBottom: 8,
  },
  saveButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
  savedOwnerText: {
    fontSize: 14,
    color: '#666',
    fontStyle: 'italic',
  },
  ownerInfo: {
    fontSize: 14,
    color: '#666',
    marginBottom: 12,
  },
  submitButton: {
    backgroundColor: '#34C759',
    paddingHorizontal: 24,
    paddingVertical: 12,
    borderRadius: 8,
    alignItems: 'center',
  },
  submitButtonDisabled: {
    opacity: 0.6,
  },
  submitButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
  offlineContainer: {
    padding: 16,
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
    marginBottom: 16,
    alignItems: 'center',
  },
  offlineMessage: {
    fontSize: 14,
    color: '#856404',
    marginBottom: 12,
    textAlign: 'center',
  },
  retryButton: {
    backgroundColor: '#FFC107',
    paddingHorizontal: 20,
    paddingVertical: 8,
    borderRadius: 6,
  },
  retryButtonText: {
    color: '#000',
    fontSize: 14,
    fontWeight: '600',
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
  emptyMessage: {
    fontSize: 14,
    color: '#666',
    fontStyle: 'italic',
    textAlign: 'center',
    padding: 16,
  },
  separator: {
    height: 8,
  },
});
