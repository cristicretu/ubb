type LogType = 'info' | 'error' | 'success';

export const log = (message: string, type: LogType = 'info'): void => {
  const ts = new Date().toISOString();
  const tag = type.toUpperCase();
  
  if (type === 'error') {
    console.error(`[${ts}] [${tag}] ${message}`);
  } else {
    console.log(`[${ts}] [${tag}] ${message}`);
  }
};
