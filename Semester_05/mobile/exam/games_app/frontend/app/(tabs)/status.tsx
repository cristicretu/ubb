import React, { useState, useEffect } from 'react';
import { View, StyleSheet, FlatList, Alert } from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getAllGames } from '@/utils/api';
import { getLocalGames, saveGames } from '@/utils/storage';
import { Game } from '@/types/game';
import { useWebSocket } from '@/hooks/useWebSocket';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import LoadingSpinner from '@/components/LoadingSpinner';
import { log } from '@/utils/logger';

export default function StatusSection() {
  const [games, setGames] = useState<Game[]>([]);
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
    loadGames();
  }, [isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name || message.id) {
        Alert.alert('New Game Added', `A new game has been added to the system.`);
        loadGames();
      }
    },
  });

  const loadGames = async () => {
    const cachedGames = await getLocalGames();
    const topCached = cachedGames
      .sort((a, b) => b.popularityScore - a.popularityScore)
      .slice(0, 10);
    
    if (topCached.length > 0) {
      setGames(topCached);
    }

    if (!isOnline) {
      return;
    }

    setLoading(true);
    const response = await getAllGames();
    setLoading(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      const sorted = response.data
        .sort((a, b) => b.popularityScore - a.popularityScore)
        .slice(0, 10);
      setGames(sorted);
      await saveGames(response.data);
    }
  };

  const renderGameItem = ({ item }: { item: Game }) => (
    <ThemedView style={styles.gameItem}>
      <ThemedText type="defaultSemiBold" style={styles.gameName}>
        {item.name}
      </ThemedText>
      <ThemedText style={styles.gameScore}>
        Popularity Score: {item.popularityScore}
      </ThemedText>
    </ThemedView>
  );

  return (
    <ThemedView style={styles.container}>
      {!isOnline && (
        <View style={styles.offlineContainer}>
          <ThemedText style={styles.offlineMessage}>
            Offline - Showing cached data
          </ThemedText>
        </View>
      )}
      {loading ? (
        <LoadingSpinner visible={true} message="Loading games..." />
      ) : games.length === 0 ? (
        <View style={styles.emptyContainer}>
          <ThemedText style={styles.emptyMessage}>No games found</ThemedText>
        </View>
      ) : (
        <FlatList
          data={games}
          renderItem={renderGameItem}
          keyExtractor={(item) => item.id.toString()}
          contentContainerStyle={styles.listContent}
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
  gameItem: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
    marginBottom: 12,
  },
  gameName: {
    fontSize: 18,
    marginBottom: 8,
  },
  gameScore: {
    fontSize: 14,
    color: '#666',
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
