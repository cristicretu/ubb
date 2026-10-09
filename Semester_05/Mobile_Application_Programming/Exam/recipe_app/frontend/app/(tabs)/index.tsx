import React, { useState, useEffect } from 'react';
import {
  View,
  StyleSheet,
  FlatList,
  Alert,
  TouchableOpacity,
  TextInput,
  ScrollView,
  ActivityIndicator,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { getTypes, getRecipesByType, createRecipe, deleteRecipe } from '@/utils/api';
import { saveTypes, getLocalTypes, saveRecipes, getLocalRecipes } from '@/utils/storage';
import { Recipe } from '@/types/recipe';
import { useWebSocket } from '@/hooks/useWebSocket';
import { log } from '@/utils/logger';
import LoadingSpinner from '@/components/LoadingSpinner';
import { ThemedText } from '@/components/themed-text';
import { ThemedView } from '@/components/themed-view';

export default function MainSection() {
  const [types, setTypes] = useState<string[]>([]);
  const [selectedType, setSelectedType] = useState<string | null>(null);
  const [recipes, setRecipes] = useState<Recipe[]>([]);
  const [loadingTypes, setLoadingTypes] = useState(false);
  const [loadingRecipes, setLoadingRecipes] = useState(false);
  const [isOnline, setIsOnline] = useState(false);
  const [isOfflineTypes, setIsOfflineTypes] = useState(false);
  const [deletingIds, setDeletingIds] = useState<Set<number>>(new Set());
  const [formData, setFormData] = useState({
    name: '',
    details: '',
    time: '',
    type: '',
    rating: '',
  });
  const [submitting, setSubmitting] = useState(false);

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
    loadTypes();
  }, []);

  useEffect(() => {
    if (selectedType) {
      loadRecipes(selectedType);
    } else {
      setRecipes([]);
    }
  }, [selectedType]);

  useWebSocket({
    onMessage: (message: any) => {
      if (message.name && message.type && message.time !== undefined) {
        Alert.alert(
          'New Recipe',
          `New Recipe: ${message.name}, Type: ${message.type}, Time: ${message.time}s`
        );
        if (selectedType === message.type) {
          loadRecipes(message.type);
        }
        loadTypes();
      }
    },
  });

  const loadTypes = async () => {
    setLoadingTypes(true);
    setIsOfflineTypes(false);

    if (isOnline) {
      const response = await getTypes();
      setLoadingTypes(false);

      if (response.error) {
        const cached = await getLocalTypes();
        if (cached.length > 0) {
          setTypes(cached);
          setIsOfflineTypes(true);
        } else {
          Alert.alert('Error', response.error);
        }
        return;
      }

      if (response.data) {
        setTypes(response.data);
        await saveTypes(response.data);
      }
    } else {
      const cached = await getLocalTypes();
      setTypes(cached);
      setLoadingTypes(false);
      setIsOfflineTypes(cached.length > 0);
    }
  };

  const loadRecipes = async (type: string) => {
    setLoadingRecipes(true);

    if (isOnline) {
      const response = await getRecipesByType(type);
      setLoadingRecipes(false);

      if (response.error) {
        const cached = await getLocalRecipes(type);
        if (cached.length > 0) {
          setRecipes(cached);
        } else {
          Alert.alert('Error', response.error);
        }
        return;
      }

      if (response.data) {
        setRecipes(response.data);
        await saveRecipes(type, response.data);
      }
    } else {
      const cached = await getLocalRecipes(type);
      setRecipes(cached);
      setLoadingRecipes(false);
    }
  };

  const handleSubmit = async () => {
    if (!isOnline) {
      Alert.alert('Offline', 'This operation requires an internet connection');
      return;
    }

    const time = parseInt(formData.time);
    const rating = parseFloat(formData.rating);

    if (!formData.name || !formData.details || !formData.time || !formData.type || !formData.rating) {
      Alert.alert('Error', 'Please fill in all fields');
      return;
    }

    if (isNaN(time) || time <= 0) {
      Alert.alert('Error', 'Time must be a positive number');
      return;
    }

    if (isNaN(rating) || rating < 0 || rating > 5) {
      Alert.alert('Error', 'Rating must be between 0 and 5');
      return;
    }

    setSubmitting(true);
    const response = await createRecipe({
      name: formData.name,
      details: formData.details,
      time,
      type: formData.type,
      rating,
    });
    setSubmitting(false);

    if (response.error) {
      Alert.alert('Error', response.error);
      return;
    }

    Alert.alert('Success', 'Recipe created successfully');
    setFormData({
      name: '',
      details: '',
      time: '',
      type: '',
      rating: '',
    });

    if (selectedType === formData.type) {
      loadRecipes(formData.type);
    }
    loadTypes();
  };

  const handleDelete = async (recipe: Recipe) => {
    if (!isOnline) {
      Alert.alert('Offline', 'This operation requires an internet connection');
      return;
    }

    Alert.alert(
      'Delete Recipe',
      `Are you sure you want to delete "${recipe.name}"?`,
      [
        { text: 'Cancel', style: 'cancel' },
        {
          text: 'Delete',
          style: 'destructive',
          onPress: async () => {
            setDeletingIds((prev) => new Set(prev).add(recipe.id));
            const response = await deleteRecipe(recipe.id);
            setDeletingIds((prev) => {
              const next = new Set(prev);
              next.delete(recipe.id);
              return next;
            });

            if (response.error) {
              Alert.alert('Error', response.error);
              return;
            }

            Alert.alert('Success', 'Recipe deleted successfully');
            if (selectedType) {
              loadRecipes(selectedType);
            }
            loadTypes();
          },
        },
      ]
    );
  };

  const renderTypeItem = ({ item }: { item: string }) => {
    const isSelected = selectedType === item;
    return (
      <TouchableOpacity
        style={[styles.typeChip, isSelected && styles.typeChipSelected]}
        onPress={() => setSelectedType(isSelected ? null : item)}>
        <ThemedText style={[styles.typeChipText, isSelected && styles.typeChipTextSelected]}>
          {item}
        </ThemedText>
      </TouchableOpacity>
    );
  };

  const renderRecipeItem = ({ item }: { item: Recipe }) => {
    const isDeleting = deletingIds.has(item.id);
    return (
      <ThemedView style={styles.recipeContainer}>
        <View style={styles.recipeInfo}>
          <ThemedText type="defaultSemiBold" style={styles.recipeName}>
            {item.name}
          </ThemedText>
          <ThemedText style={styles.recipeDetails}>{item.details}</ThemedText>
          <View style={styles.recipeMeta}>
            <ThemedText style={styles.recipeMetaText}>Time: {item.time}s</ThemedText>
            <ThemedText style={styles.recipeMetaText}>Rating: {item.rating}</ThemedText>
          </View>
        </View>
        {isOnline && (
          <TouchableOpacity
            style={[styles.deleteButton, isDeleting && styles.buttonDisabled]}
            onPress={() => handleDelete(item)}
            disabled={isDeleting}>
            {isDeleting ? (
              <ActivityIndicator size="small" color="#fff" />
            ) : (
              <ThemedText style={styles.deleteButtonText}>DELETE</ThemedText>
            )}
          </TouchableOpacity>
        )}
      </ThemedView>
    );
  };

  return (
    <ThemedView style={styles.container}>
      <ScrollView style={styles.scrollView} contentContainerStyle={styles.scrollContent}>
        <View style={styles.section}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            Recipe Types
          </ThemedText>
          {isOfflineTypes && (
            <View style={styles.offlineBanner}>
              <ThemedText style={styles.offlineText}>Showing cached types</ThemedText>
              <TouchableOpacity onPress={loadTypes} style={styles.retryButton}>
                <ThemedText style={styles.retryButtonText}>Retry</ThemedText>
              </TouchableOpacity>
            </View>
          )}
          {loadingTypes ? (
            <LoadingSpinner visible={true} message="Loading types..." />
          ) : (
            <FlatList
              data={types}
              renderItem={renderTypeItem}
              keyExtractor={(item) => item}
              horizontal
              showsHorizontalScrollIndicator={false}
              contentContainerStyle={styles.typesList}
              ItemSeparatorComponent={() => <View style={styles.typeSeparator} />}
            />
          )}
        </View>

        {selectedType && (
          <View style={styles.section}>
            <ThemedText type="subtitle" style={styles.sectionTitle}>
              Recipes: {selectedType}
            </ThemedText>
            {loadingRecipes ? (
              <LoadingSpinner visible={true} message="Loading recipes..." />
            ) : (
              <>
                {recipes.length === 0 ? (
                  <View style={styles.emptyContainer}>
                    <ThemedText style={styles.emptyMessage}>No recipes found</ThemedText>
                  </View>
                ) : (
                  <FlatList
                    data={recipes}
                    renderItem={renderRecipeItem}
                    keyExtractor={(item) => item.id.toString()}
                    scrollEnabled={false}
                    ItemSeparatorComponent={() => <View style={styles.recipeSeparator} />}
                  />
                )}
              </>
            )}
          </View>
        )}

        <View style={styles.section}>
          <ThemedText type="subtitle" style={styles.sectionTitle}>
            Add Recipe
          </ThemedText>
          {!isOnline && (
            <View style={styles.offlineBanner}>
              <ThemedText style={styles.offlineText}>This section requires an internet connection</ThemedText>
            </View>
          )}
          <View style={styles.form}>
            <TextInput
              style={styles.input}
              placeholder="Name"
              value={formData.name}
              onChangeText={(text) => setFormData({ ...formData, name: text })}
              autoCapitalize="none"
              editable={isOnline}
            />
            <TextInput
              style={styles.input}
              placeholder="Details"
              value={formData.details}
              onChangeText={(text) => setFormData({ ...formData, details: text })}
              autoCapitalize="none"
              multiline
              editable={isOnline}
            />
            <TextInput
              style={styles.input}
              placeholder="Time (seconds)"
              value={formData.time}
              onChangeText={(text) => setFormData({ ...formData, time: text })}
              keyboardType="numeric"
              autoCapitalize="none"
              editable={isOnline}
            />
            <TextInput
              style={styles.input}
              placeholder="Type"
              value={formData.type}
              onChangeText={(text) => setFormData({ ...formData, type: text })}
              autoCapitalize="none"
              editable={isOnline}
            />
            <TextInput
              style={styles.input}
              placeholder="Rating (0-5)"
              value={formData.rating}
              onChangeText={(text) => setFormData({ ...formData, rating: text })}
              keyboardType="decimal-pad"
              autoCapitalize="none"
              editable={isOnline}
            />
            <TouchableOpacity
              style={[styles.submitButton, (!isOnline || submitting) && styles.buttonDisabled]}
              onPress={handleSubmit}
              disabled={!isOnline || submitting}>
              {submitting ? (
                <ActivityIndicator size="small" color="#fff" />
              ) : (
                <ThemedText style={styles.submitButtonText}>Add Recipe</ThemedText>
              )}
            </TouchableOpacity>
          </View>
        </View>
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
    marginBottom: 12,
  },
  offlineBanner: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 12,
    backgroundColor: '#FFF3CD',
    borderRadius: 8,
    marginBottom: 12,
  },
  offlineText: {
    fontSize: 14,
    color: '#856404',
    flex: 1,
  },
  retryButton: {
    paddingHorizontal: 12,
    paddingVertical: 6,
    backgroundColor: '#856404',
    borderRadius: 4,
  },
  retryButtonText: {
    color: '#fff',
    fontSize: 12,
    fontWeight: '600',
  },
  typesList: {
    paddingVertical: 8,
  },
  typeChip: {
    paddingHorizontal: 16,
    paddingVertical: 8,
    backgroundColor: '#E5E5E5',
    borderRadius: 20,
  },
  typeChipSelected: {
    backgroundColor: '#007AFF',
  },
  typeChipText: {
    fontSize: 14,
    color: '#000',
  },
  typeChipTextSelected: {
    color: '#fff',
  },
  typeSeparator: {
    width: 8,
  },
  recipeContainer: {
    padding: 16,
    borderRadius: 8,
    backgroundColor: '#F5F5F5',
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  recipeInfo: {
    flex: 1,
  },
  recipeName: {
    fontSize: 16,
    marginBottom: 4,
  },
  recipeDetails: {
    fontSize: 14,
    color: '#666',
    marginBottom: 8,
  },
  recipeMeta: {
    flexDirection: 'row',
    gap: 16,
  },
  recipeMetaText: {
    fontSize: 12,
    color: '#999',
  },
  recipeSeparator: {
    height: 8,
  },
  deleteButton: {
    backgroundColor: '#FF3B30',
    paddingVertical: 8,
    paddingHorizontal: 12,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
    minWidth: 80,
  },
  deleteButtonText: {
    color: '#fff',
    fontSize: 12,
    fontWeight: '600',
  },
  buttonDisabled: {
    opacity: 0.6,
  },
  emptyContainer: {
    padding: 40,
    alignItems: 'center',
  },
  emptyMessage: {
    fontSize: 14,
    color: '#999',
  },
  form: {
    gap: 12,
  },
  input: {
    borderWidth: 1,
    borderColor: '#E5E5E5',
    borderRadius: 8,
    padding: 12,
    fontSize: 16,
    backgroundColor: '#fff',
    minHeight: 44,
  },
  submitButton: {
    backgroundColor: '#34C759',
    paddingVertical: 12,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
    marginTop: 8,
  },
  submitButtonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: '600',
  },
});
