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
import { getAvailableGames, bookGame } from '@/utils/api';
import { getUserName } from '@/utils/storage';
import { Game } from '@/types/game';
import { useWebSocket } from '@/hooks/useWebSocket';
import LoadingSpinner from '@/components/LoadingSpinner';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import { log } from '@/utils/logger';

export default function SelectionSection() {
  const [games, setGames] = useState<Game[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [isOnline, setIsOnline] = useState(true);
  const [refreshing, setRefreshing] = useState(false);
  const [borrowingId, setBorrowingId] = useState<number | null>(null);

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
    if (isOnline) {
      loadGames();
    }
  }, [isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name && message.size !== undefined) {
        Alert.alert(
          'New Game Added',
          `New game available: ${message.name}\nSize: ${message.size}MB\nPopularity Score: ${message.popularityScore || 0}`
        );
        // Refresh the list to show the new game
        if (isOnline) {
          loadGames();
        }
      }
    },
  });

  const loadGames = async () => {
    if (!isOnline) {
      return;
    }

    setIsLoading(true);
    log('Loading available games...', 'info');

    try {
      const response = await getAvailableGames();
      if (response.error) {
        log(`Error: ${response.error}`, 'error');
        Alert.alert('Error', response.error);
      } else if (response.data) {
        setGames(response.data);
        log(`Loaded ${response.data.length} available games`, 'success');
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
    await loadGames();
    setRefreshing(false);
  }, [isOnline]);

  const handleBorrow = async (game: Game) => {
    if (!isOnline) {
      Alert.alert('Offline', 'This section requires internet connection.');
      return;
    }

    // Check if user name is saved
    const userName = await getUserName();
    if (!userName) {
      Alert.alert('Error', 'Please save your name in User Section first');
      return;
    }

    setBorrowingId(game.id);
    log(`Borrowing game: ${game.name}`, 'info');

    try {
      const response = await bookGame(game.id, userName);
      if (response.error) {
        log(`Error: ${response.error}`, 'error');
        Alert.alert('Error', response.error);
      } else {
        log(`Borrowed game: ${game.name}`, 'success');
        Alert.alert('Success', `Game "${game.name}" borrowed successfully`);
        // Refresh the list - the game should disappear since it's no longer available
        await loadGames();
      }
    } catch (error) {
      const msg = error instanceof Error ? error.message : 'Unknown error';
      log(`Error: ${msg}`, 'error');
      Alert.alert('Error', msg);
    } finally {
      setBorrowingId(null);
    }
  };

  const renderGameItem = ({ item }: { item: Game }) => (
    <ThemedView style={styles.gameItem}>
      <View style={styles.gameContent}>
        <ThemedText type="defaultSemiBold" style={styles.gameName}>
          {item.name}
        </ThemedText>
        <ThemedText style={styles.gameDetail}>Size: {item.size}MB</ThemedText>
        <ThemedText style={styles.gameDetail}>
          Popularity Score: {item.popularityScore}
        </ThemedText>
      </View>
      <TouchableOpacity
        style={[
          styles.borrowButton,
          borrowingId === item.id && styles.borrowButtonDisabled,
        ]}
        onPress={() => handleBorrow(item)}
        disabled={borrowingId === item.id || !isOnline}>
        {borrowingId === item.id ? (
          <ActivityIndicator size="small" color="#fff" />
        ) : (
          <ThemedText style={styles.borrowButtonText}>Borrow</ThemedText>
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
        <LoadingSpinner visible={true} message="Loading available games..." />
      ) : (
        <FlatList
          data={games}
          renderItem={renderGameItem}
          keyExtractor={(item) => item.id.toString()}
          contentContainerStyle={styles.listContent}
          ListEmptyComponent={
            <ThemedView style={styles.emptyContainer}>
              <ThemedText style={styles.emptyText}>No available games found</ThemedText>
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
  gameItem: {
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
  gameContent: {
    flex: 1,
    marginRight: 12,
  },
  gameName: {
    fontSize: 18,
    marginBottom: 8,
  },
  gameDetail: {
    fontSize: 14,
    marginBottom: 4,
    color: '#666',
  },
  borrowButton: {
    backgroundColor: '#007AFF',
    paddingHorizontal: 16,
    paddingVertical: 8,
    borderRadius: 6,
    minWidth: 80,
    alignItems: 'center',
    justifyContent: 'center',
  },
  borrowButtonDisabled: {
    opacity: 0.6,
  },
  borrowButtonText: {
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
