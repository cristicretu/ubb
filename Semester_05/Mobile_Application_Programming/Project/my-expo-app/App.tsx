import { StatusBar } from 'expo-status-bar';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
import { CarsProvider } from './contexts/CarsContext';
import { CarListView } from './components/CarListView';

import './global.css';

export default function App() {
  return (
    <GestureHandlerRootView style={{ flex: 1 }}>
      <CarsProvider>
        <CarListView />
        <StatusBar style="auto" />
      </CarsProvider>
    </GestureHandlerRootView>
  );
}
