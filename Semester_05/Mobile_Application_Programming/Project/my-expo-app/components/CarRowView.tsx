import React from 'react';
import { View, Text, TouchableOpacity } from 'react-native';
import { Car } from '../types/Car';

interface CarRowViewProps {
  car: Car;
  onPress: () => void;
}

export const CarRowView: React.FC<CarRowViewProps> = ({ car, onPress }) => {
  return (
    <TouchableOpacity
      onPress={onPress}
      className="flex-row items-center justify-between py-3 px-4 bg-white border-b border-gray-300"
      activeOpacity={0.7}
    >
      <View className="flex-1">
        <View className="flex-row">
          <Text className="text-base font-normal">{car.make} </Text>
          <Text className="text-base font-normal">{car.model}</Text>
        </View>
        <Text className="text-sm text-gray-500 mt-0.5">{car.licensePlate}</Text>
      </View>
      <View className="ml-2">
        <Text className="text-gray-400 text-xl font-semibold">›</Text>
      </View>
    </TouchableOpacity>
  );
};

