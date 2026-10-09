import React, { useState } from 'react';
import {
  View,
  Text,
  FlatList,
  TouchableOpacity,
  Modal,
  Alert,
  Platform,
  ActionSheetIOS,
} from 'react-native';
import { useCars } from '../contexts/CarsContext';
import { CarRowView } from './CarRowView';
import { CarFormView } from './CarFormView';
import { Car } from '../types/Car';
import { Swipeable } from 'react-native-gesture-handler';

export const CarListView: React.FC = () => {
  const { cars, addCar, updateCar, deleteCar, isOnline, isSyncing, syncNow } = useCars();
  const [selectedCar, setSelectedCar] = useState<Car | null>(null);
  const [showingForm, setShowingForm] = useState(false);

  const handleAddCar = () => {
    setSelectedCar(null);
    setShowingForm(true);
  };

  const handleEditCar = (car: Car) => {
    setSelectedCar(car);
    setShowingForm(true);
  };

  const handleSaveCar = async (car: Car | Omit<Car, 'id'>) => {
    try {
      if (selectedCar) {
        await updateCar(car as Car);
      } else {
        await addCar(car as Omit<Car, 'id'>);
      }
      setShowingForm(false);
      setSelectedCar(null);
    } catch (error) {
      console.error('Error saving car:', error);
    }
  };

  const handleDeleteCar = (car: Car) => {
    if (Platform.OS === 'ios') {
      ActionSheetIOS.showActionSheetWithOptions(
        {
          title: 'Delete Car',
          message: `Are you sure you want to delete ${car.make} ${car.model}?`,
          options: ['Cancel', 'Delete'],
          destructiveButtonIndex: 1,
          cancelButtonIndex: 0,
        },
        async (buttonIndex) => {
          if (buttonIndex === 1) {
            try {
              await deleteCar(car.id);
            } catch (error) {
              console.error('Error deleting car:', error);
            }
          }
        }
      );
    } else {
      Alert.alert(
        'Delete Car',
        `Are you sure you want to delete ${car.make} ${car.model}?`,
        [
          {
            text: 'Cancel',
            style: 'cancel',
          },
          {
            text: 'Delete',
            style: 'destructive',
            onPress: async () => {
              try {
                await deleteCar(car.id);
              } catch (error) {
                console.error('Error deleting car:', error);
              }
            },
          },
        ]
      );
    }
  };

  const renderRightActions = (car: Car) => {
    return (
      <TouchableOpacity
        onPress={() => handleDeleteCar(car)}
        className="bg-red-500 justify-center items-center px-6"
        activeOpacity={0.7}
      >
        <Text className="text-white font-semibold">Delete</Text>
      </TouchableOpacity>
    );
  };

  const renderCarItem = ({ item }: { item: Car }) => {
    return (
      <Swipeable renderRightActions={() => renderRightActions(item)}>
        <CarRowView car={item} onPress={() => handleEditCar(item)} />
      </Swipeable>
    );
  };

  return (
    <View className="flex-1 bg-gray-50">
      <View className="bg-white border-b border-gray-300 pt-14 pb-2 px-4 shadow-sm">
        <View className="flex-row justify-between items-center">
          <View className="flex-1">
            <Text className="text-3xl font-bold">My Cars</Text>
            <View className="flex-row items-center mt-1">
              <View className={`w-2 h-2 rounded-full mr-2 ${isOnline ? 'bg-green-500' : 'bg-gray-400'}`} />
              <Text className="text-xs text-gray-600">
                {isOnline ? (isSyncing ? 'Syncing...' : 'Online') : 'Offline'}
              </Text>
              {isOnline && !isSyncing && (
                <TouchableOpacity onPress={syncNow} className="ml-2">
                  <Text className="text-xs text-blue-500">↻ Sync</Text>
                </TouchableOpacity>
              )}
            </View>
          </View>
          <TouchableOpacity onPress={handleAddCar} activeOpacity={0.6}>
            <Text className="text-blue-500 text-3xl font-light">+</Text>
          </TouchableOpacity>
        </View>
      </View>

      <FlatList
        data={cars}
        renderItem={renderCarItem}
        keyExtractor={(item) => item.id}
        contentContainerStyle={{ flexGrow: 1 }}
        className="bg-white"
      />

      <Modal
        visible={showingForm}
        animationType="slide"
        presentationStyle="pageSheet"
      >
        <CarFormView
          car={selectedCar || undefined}
          onSave={handleSaveCar}
          onCancel={() => {
            setShowingForm(false);
            setSelectedCar(null);
          }}
        />
      </Modal>
    </View>
  );
};

