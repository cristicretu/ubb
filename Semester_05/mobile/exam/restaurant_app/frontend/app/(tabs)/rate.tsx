import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  FlatList,
  TouchableOpacity,
  Alert,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';
import LoadingSpinner from '@/components/LoadingSpinner';
import { getLowRated, incrementRating } from '@/utils/api';
import { Recipe } from '@/types/recipe';
import { useWebSocket } from '@/hooks/useWebSocket';
import { log } from '@/utils/logger';

export default function RateSection() {
  const [isOnline, setIsOnline] = useState(false);
  const [recipes, setRecipes] = useState<Recipe[]>([]);
  const [loading, setLoading] = useState(false);
  const [incrementingIds, setIncrementingIds] = useState<Set<number>>(new Set());

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
    loadRecipes();
  }, []);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name && message.type) {
        Alert.alert('New Recipe Added', `New recipe: ${message.name} (${message.type})`);
        loadRecipes();
      }
    },
  });

  const loadRecipes = async () => {
    setLoading(true);
    const response = await getLowRated();
    setLoading(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      const sorted = [...response.data]
        .sort((a, b) => a.rating - b.rating)
        .slice(0, 10);
      setRecipes(sorted);
    }
  };

  const handleIncrement = async (recipeId: number) => {
    if (!isOnline) {
      Alert.alert('Offline', 'Cannot increment rating while offline');
      return;
    }

    const recipe = recipes.find(r => r.id === recipeId);
    setIncrementingIds((prev) => new Set(prev).add(recipeId));
    const response = await incrementRating(recipeId);
    setIncrementingIds((prev) => {
      const next = new Set(prev);
      next.delete(recipeId);
      return next;
    });

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    if (response.data) {
      Alert.alert('Success', `${recipe?.name} rating updated to ${response.data.rating}`);
    }

    await loadRecipes();
  };

  const renderItem = ({ item }: { item: Recipe }) => {
    const isIncrementing = incrementingIds.has(item.id);
    return (
      <ThemedView style={styles.itemContainer}>
        <ThemedText type="defaultSemiBold" style={styles.itemName}>
          {item.name}
        </ThemedText>
        <ThemedText style={styles.itemDetails}>{item.details}</ThemedText>
        <View style={styles.itemMeta}>
          <ThemedText style={styles.itemType}>Type: {item.type}</ThemedText>
          <ThemedText style={styles.itemRating}>Rating: {item.rating}</ThemedText>
        </View>
        <TouchableOpacity
          style={[styles.incrementButton, !isOnline && styles.incrementButtonDisabled]}
          onPress={() => handleIncrement(item.id)}
          disabled={!isOnline || isIncrementing}>
          {isIncrementing ? (
            <ActivityIndicator size="small" color="#fff" />
          ) : (
            <ThemedText style={styles.incrementButtonText}>Rate/Increment</ThemedText>
          )}
        </TouchableOpacity>
      </ThemedView>
    );
  };

  return (
    <View style={styles.container}>
      {loading ? (
        <LoadingSpinner visible={true} message="Loading recipes..." />
      ) : recipes.length === 0 ? (
        <ThemedText style={styles.emptyMessage}>No recipes found</ThemedText>
      ) : (
        <FlatList
          data={recipes}
          renderItem={renderItem}
          keyExtractor={(item) => item.id.toString()}
          contentContainerStyle={styles.listContent}
          ItemSeparatorComponent={() => <View style={styles.separator} />}
        />
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  listContent: {
    padding: 16,
  },
  itemContainer: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
  },
  itemName: {
    fontSize: 18,
    marginBottom: 8,
  },
  itemDetails: {
    fontSize: 14,
    color: '#666',
    marginBottom: 12,
  },
  itemMeta: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginBottom: 12,
  },
  itemType: {
    fontSize: 14,
    color: '#666',
  },
  itemRating: {
    fontSize: 14,
    color: '#666',
    fontWeight: '600',
  },
  incrementButton: {
    backgroundColor: '#007AFF',
    paddingHorizontal: 24,
    paddingVertical: 12,
    borderRadius: 8,
    alignItems: 'center',
  },
  incrementButtonDisabled: {
    opacity: 0.5,
  },
  incrementButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
  separator: {
    height: 12,
  },
  emptyMessage: {
    fontSize: 14,
    color: '#666',
    fontStyle: 'italic',
    textAlign: 'center',
    padding: 16,
    marginTop: 32,
  },
});
