type LogLevel = 'info' | 'success' | 'error';

export const log = (message: string, level: LogLevel = 'info') => {
  const timestamp = new Date().toISOString();
  console.log(`[${timestamp}] [${level.toUpperCase()}] ${message}`);
};
