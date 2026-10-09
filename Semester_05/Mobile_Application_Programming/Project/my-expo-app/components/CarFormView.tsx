import React, { useState } from 'react';
import {
  View,
  Text,
  TextInput,
  ScrollView,
  TouchableOpacity,
  Alert,
  Platform,
} from 'react-native';
import DateTimePicker from '@react-native-community/datetimepicker';
import { Car } from '../types/Car';

interface CarFormViewProps {
  car?: Car;
  onSave: (car: Car | Omit<Car, 'id'>) => Promise<void>;
  onCancel: () => void;
}

export const CarFormView: React.FC<CarFormViewProps> = ({
  car,
  onSave,
  onCancel,
}) => {
  const [make, setMake] = useState(car?.make || '');
  const [model, setModel] = useState(car?.model || '');
  const [year, setYear] = useState(car?.year.toString() || '');
  const [licensePlate, setLicensePlate] = useState(car?.licensePlate || '');
  const [currentMileage, setCurrentMileage] = useState(
    car?.currentMileage.toString() || ''
  );
  const [lastITPDate, setLastITPDate] = useState(car?.lastITPDate || new Date());
  const [nextITPDate, setNextITPDate] = useState(car?.nextITPDate || new Date());
  const [rovinietaExpiryDate, setRovinietaExpiryDate] = useState(
    car?.rovinietaExpiryDate || new Date()
  );
  const [carDescription, setCarDescription] = useState(
    car?.carDescription || ''
  );

  const [showLastITPPicker, setShowLastITPPicker] = useState(false);
  const [showNextITPPicker, setShowNextITPPicker] = useState(false);
  const [showRovinietaPicker, setShowRovinietaPicker] = useState(false);

  const formatDate = (date: Date) => {
    return date.toLocaleDateString('en-US', {
      year: 'numeric',
      month: 'short',
      day: 'numeric',
    });
  };

  const handleSave = async () => {
    // Validation
    if (!make.trim()) {
      Alert.alert('Validation Error', 'Make is required');
      return;
    }

    if (!model.trim()) {
      Alert.alert('Validation Error', 'Model is required');
      return;
    }

    const yearInt = parseInt(year);
    if (!yearInt || yearInt < 1900 || yearInt > 2100) {
      Alert.alert('Validation Error', 'Please enter a valid year');
      return;
    }

    if (!licensePlate.trim()) {
      Alert.alert('Validation Error', 'License plate is required');
      return;
    }

    const mileageDouble = parseFloat(currentMileage);
    if (isNaN(mileageDouble) || mileageDouble < 0) {
      Alert.alert('Validation Error', 'Please enter a valid mileage');
      return;
    }

    if (car) {
      const updatedCar: Car = {
        id: car.id,
        make: make.trim(),
        model: model.trim(),
        year: yearInt,
        licensePlate: licensePlate.trim(),
        currentMileage: mileageDouble,
        lastITPDate,
        nextITPDate,
        rovinietaExpiryDate,
        carDescription: carDescription.trim() || undefined,
      };
      await onSave(updatedCar);
    } else {
      const newCarData: Omit<Car, 'id'> = {
        make: make.trim(),
        model: model.trim(),
        year: yearInt,
        licensePlate: licensePlate.trim(),
        currentMileage: mileageDouble,
        lastITPDate,
        nextITPDate,
        rovinietaExpiryDate,
        carDescription: carDescription.trim() || undefined,
      };
      await onSave(newCarData);
    }
  };

  return (
    <View className="flex-1 bg-gray-100">
      {/* Header */}
      <View className="bg-gray-50 border-b border-gray-300 pt-12 pb-2 px-4">
        <View className="flex-row justify-between items-center">
          <TouchableOpacity onPress={onCancel} activeOpacity={0.6}>
            <Text className="text-blue-500 text-base">Cancel</Text>
          </TouchableOpacity>
          <Text className="text-base font-semibold">
            {car ? 'Edit Car' : 'Add Car'}
          </Text>
          <TouchableOpacity onPress={handleSave} activeOpacity={0.6}>
            <Text className="text-blue-500 text-base font-semibold">Save</Text>
          </TouchableOpacity>
        </View>
      </View>

      <ScrollView className="flex-1">
        {/* Car Information Section */}
        <View className="mt-6">
          <Text className="px-4 pb-2 text-xs text-gray-500 uppercase tracking-wide">
            Car Information
          </Text>
          <View className="bg-white">
            {/* Make */}
            <View className="border-b border-gray-300 px-4 py-3 flex-row items-center justify-between">
              <Text className="text-base text-gray-900 w-40">Make</Text>
              <TextInput
                value={make}
                onChangeText={setMake}
                placeholder="Make"
                className="text-base flex-1 text-right"
                placeholderTextColor="#999"
              />
            </View>

            {/* Model */}
            <View className="border-b border-gray-300 px-4 py-3 flex-row items-center justify-between">
              <Text className="text-base text-gray-900 w-40">Model</Text>
              <TextInput
                value={model}
                onChangeText={setModel}
                placeholder="Model"
                className="text-base flex-1 text-right"
                placeholderTextColor="#999"
              />
            </View>

            {/* Year */}
            <View className="border-b border-gray-300 px-4 py-3 flex-row items-center justify-between">
              <Text className="text-base text-gray-900 w-40">Year</Text>
              <TextInput
                value={year}
                onChangeText={setYear}
                placeholder="Year"
                keyboardType="number-pad"
                className="text-base flex-1 text-right"
                placeholderTextColor="#999"
              />
            </View>

            {/* License Plate */}
            <View className="border-b border-gray-300 px-4 py-3 flex-row items-center justify-between">
              <Text className="text-base text-gray-900 w-40">License Plate</Text>
              <TextInput
                value={licensePlate}
                onChangeText={setLicensePlate}
                placeholder="License Plate"
                className="text-base flex-1 text-right"
                placeholderTextColor="#999"
              />
            </View>

            {/* Current Mileage */}
            <View className="border-b border-gray-300 px-4 py-3 flex-row items-center justify-between">
              <Text className="text-base text-gray-900 w-40">Current Mileage</Text>
              <TextInput
                value={currentMileage}
                onChangeText={setCurrentMileage}
                placeholder="Current Mileage"
                keyboardType="decimal-pad"
                className="text-base flex-1 text-right"
                placeholderTextColor="#999"
              />
            </View>

            {/* Last ITP Date */}
            <View className="border-b border-gray-300 px-4 py-3 flex-row items-center justify-between">
              <Text className="text-base text-gray-900 w-40">Last ITP Date</Text>
              <TouchableOpacity onPress={() => setShowLastITPPicker(true)} activeOpacity={0.6}>
                <Text className="text-base text-gray-900">{formatDate(lastITPDate)}</Text>
              </TouchableOpacity>
              {showLastITPPicker && (
                <DateTimePicker
                  value={lastITPDate}
                  mode="date"
                  display={Platform.OS === 'ios' ? 'spinner' : 'default'}
                  onChange={(event, selectedDate) => {
                    setShowLastITPPicker(Platform.OS === 'ios');
                    if (selectedDate) {
                      setLastITPDate(selectedDate);
                    }
                  }}
                />
              )}
            </View>

            {/* Next ITP Date */}
            <View className="border-b border-gray-300 px-4 py-3 flex-row items-center justify-between">
              <Text className="text-base text-gray-900 w-40">Next ITP Date</Text>
              <TouchableOpacity onPress={() => setShowNextITPPicker(true)} activeOpacity={0.6}>
                <Text className="text-base text-gray-900">{formatDate(nextITPDate)}</Text>
              </TouchableOpacity>
              {showNextITPPicker && (
                <DateTimePicker
                  value={nextITPDate}
                  mode="date"
                  display={Platform.OS === 'ios' ? 'spinner' : 'default'}
                  onChange={(event, selectedDate) => {
                    setShowNextITPPicker(Platform.OS === 'ios');
                    if (selectedDate) {
                      setNextITPDate(selectedDate);
                    }
                  }}
                />
              )}
            </View>

            {/* Rovinieta Expiry Date */}
            <View className="border-b border-gray-300 px-4 py-3 flex-row items-center justify-between">
              <Text className="text-base text-gray-900 w-40">Rovinieta Expiry Date</Text>
              <TouchableOpacity onPress={() => setShowRovinietaPicker(true)} activeOpacity={0.6}>
                <Text className="text-base text-gray-900">{formatDate(rovinietaExpiryDate)}</Text>
              </TouchableOpacity>
              {showRovinietaPicker && (
                <DateTimePicker
                  value={rovinietaExpiryDate}
                  mode="date"
                  display={Platform.OS === 'ios' ? 'spinner' : 'default'}
                  onChange={(event, selectedDate) => {
                    setShowRovinietaPicker(Platform.OS === 'ios');
                    if (selectedDate) {
                      setRovinietaExpiryDate(selectedDate);
                    }
                  }}
                />
              )}
            </View>

            {/* Car Description */}
            <View className="px-4 py-3 flex-row items-center justify-between">
              <Text className="text-base text-gray-900 w-40">Car Description</Text>
              <TextInput
                value={carDescription}
                onChangeText={setCarDescription}
                placeholder="Car Description"
                className="text-base flex-1 text-right"
                placeholderTextColor="#999"
              />
            </View>
          </View>
        </View>

        {/* Save Button */}
        <View className="mt-6 bg-white">
          <TouchableOpacity
            onPress={handleSave}
            className="py-3 items-center border-b border-t border-gray-300"
            activeOpacity={0.6}
          >
            <Text className="text-blue-500 text-base font-semibold">Save</Text>
          </TouchableOpacity>
        </View>
      </ScrollView>
    </View>
  );
};

