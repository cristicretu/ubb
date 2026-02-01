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
import { createGame, getGamesByUser } from '@/utils/api';
import { saveGames, getLocalGames, savePendingGame, saveUserName, getUserName } from '@/utils/storage';
import { Game } from '@/types/game';
import { useWebSocket } from '@/hooks/useWebSocket';
import { log } from '@/utils/logger';

export default function UserSection() {
  const [isOnline, setIsOnline] = useState(false);
  const [userName, setUserName] = useState('');
  const [savedUser, setSavedUser] = useState<string | null>(null);
  const [games, setGames] = useState<Game[]>([]);
  const [loading, setLoading] = useState(false);
  const [submitting, setSubmitting] = useState(false);
  
  // Game form fields
  const [gameName, setGameName] = useState('');
  const [gameStatus, setGameStatus] = useState('');
  const [gameSize, setGameSize] = useState('');

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
    loadSavedUser();
  }, []);

  useEffect(() => {
    if (savedUser) {
      loadGames();
    }
  }, [savedUser, isOnline]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name && message.size !== undefined && message.popularityScore !== undefined) {
        Alert.alert(
          'New Game',
          `New Game: ${message.name}, Size: ${message.size}KB, Popularity: ${message.popularityScore}`
        );
        // Refresh games if it's for the current user
        if (savedUser && message.user && message.user.toLowerCase() === savedUser.toLowerCase()) {
          loadGames();
        }
      }
    },
  });

  const loadSavedUser = async () => {
    const user = await getUserName();
    if (user) {
      setSavedUser(user);
      setUserName(user);
    }
  };

  const handleSaveUser = async () => {
    const trimmed = userName.trim();
    if (!trimmed) {
      Alert.alert('Error', 'User name cannot be empty');
      return;
    }
    await saveUserName(trimmed);
    setSavedUser(trimmed);
    Alert.alert('Success', 'User name saved');
  };

  const loadGames = async () => {
    if (!savedUser) return;

    // Try to load from cache first
    const cachedGames = await getLocalGames();
    const userGames = cachedGames.filter(
      (game) => game.user.toLowerCase() === savedUser.toLowerCase()
    );
    
    if (userGames.length > 0) {
      setGames(userGames);
    }

    if (!isOnline) {
      return;
    }

    setLoading(true);
    const response = await getGamesByUser(savedUser);
    setLoading(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      setGames(response.data);
      // Cache all games (merge with existing)
      const allCached = await getLocalGames();
      const updatedGames = [...allCached];
      response.data.forEach((game) => {
        const index = updatedGames.findIndex((g) => g.id === game.id);
        if (index >= 0) {
          updatedGames[index] = game;
        } else {
          updatedGames.push(game);
        }
      });
      await saveGames(updatedGames);
    }
  };

  const handleSubmitGame = async () => {
    if (!savedUser) {
      Alert.alert('Error', 'Please save user name first');
      return;
    }

    const trimmedName = gameName.trim();
    const trimmedStatus = gameStatus.trim();
    const sizeNum = parseInt(gameSize);

    if (!trimmedName) {
      Alert.alert('Error', 'Game name is required');
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

    const newGame: Omit<Game, 'id' | 'popularityScore'> = {
      name: trimmedName,
      status: trimmedStatus,
      user: savedUser,
      size: sizeNum,
    };

    if (isOnline) {
      setSubmitting(true);
      const response = await createGame(newGame);
      setSubmitting(false);

      if (response.error) {
        Alert.alert('Error', response.error);
        // Save as pending on error
        await savePendingGame({ ...newGame, id: Date.now(), popularityScore: 0 });
        return;
      }

      if (response.data) {
        Alert.alert('Success', 'Game created');
        setGameName('');
        setGameStatus('');
        setGameSize('');
        await loadGames();
      }
    } else {
      // Offline: save as pending
      await savePendingGame({ ...newGame, id: Date.now(), popularityScore: 0 });
      Alert.alert('Offline', 'Game saved offline. It will be synced when online.');
      setGameName('');
      setGameStatus('');
      setGameSize('');
    }
  };

  const renderGameItem = ({ item }: { item: Game }) => (
    <ThemedView style={styles.gameItem}>
      <ThemedText type="defaultSemiBold" style={styles.gameName}>
        {item.name}
      </ThemedText>
      <ThemedText style={styles.gameInfo}>
        Status: {item.status} | Size: {item.size}KB | Popularity: {item.popularityScore}
      </ThemedText>
    </ThemedView>
  );

  return (
    <ScrollView style={styles.container} contentContainerStyle={styles.contentContainer}>
      {/* User Settings Section */}
      <ThemedView style={styles.section}>
        <ThemedText type="subtitle" style={styles.sectionTitle}>
          User Settings
        </ThemedText>
        <TextInput
          style={styles.input}
          placeholder="Enter user name"
          value={userName}
          onChangeText={setUserName}
          autoCapitalize="none"
        />
        <TouchableOpacity style={styles.saveButton} onPress={handleSaveUser}>
          <ThemedText style={styles.saveButtonText}>Save</ThemedText>
        </TouchableOpacity>
        {savedUser && (
          <ThemedText style={styles.savedUserText}>Saved user: {savedUser}</ThemedText>
        )}
      </ThemedView>

      {/* Game Form Section - Only show if user saved */}
      {savedUser && (
        <ThemedView style={styles.section}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            Record Game
          </ThemedText>
          <TextInput
            style={styles.input}
            placeholder="Game name"
            value={gameName}
            onChangeText={setGameName}
            autoCapitalize="none"
          />
          <TextInput
            style={styles.input}
            placeholder="Status"
            value={gameStatus}
            onChangeText={setGameStatus}
            autoCapitalize="none"
          />
          <TextInput
            style={styles.input}
            placeholder="Size (KB)"
            value={gameSize}
            onChangeText={setGameSize}
            keyboardType="numeric"
            autoCapitalize="none"
          />
          <ThemedText style={styles.userInfo}>User: {savedUser}</ThemedText>
          <TouchableOpacity
            style={[styles.submitButton, submitting && styles.submitButtonDisabled]}
            onPress={handleSubmitGame}
            disabled={submitting}>
            {submitting ? (
              <ActivityIndicator size="small" color="#fff" />
            ) : (
              <ThemedText style={styles.submitButtonText}>Submit</ThemedText>
            )}
          </TouchableOpacity>
        </ThemedView>
      )}

      {/* Games List Section */}
      {savedUser && (
        <ThemedView style={styles.section}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            My Borrowed Games
          </ThemedText>
          {!isOnline ? (
            <View style={styles.offlineContainer}>
              <ThemedText style={styles.offlineMessage}>Offline - Showing cached data</ThemedText>
              <TouchableOpacity style={styles.retryButton} onPress={loadGames}>
                <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
              </TouchableOpacity>
            </View>
          ) : null}
          {loading ? (
            <LoadingSpinner visible={true} message="Loading games..." />
          ) : games.length === 0 ? (
            <ThemedText style={styles.emptyMessage}>No games found</ThemedText>
          ) : (
            <FlatList
              data={games}
              renderItem={renderGameItem}
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
  savedUserText: {
    fontSize: 14,
    color: '#666',
    fontStyle: 'italic',
  },
  userInfo: {
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
  gameItem: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  gameName: {
    fontSize: 16,
    marginBottom: 4,
  },
  gameInfo: {
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
