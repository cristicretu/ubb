type LogType = 'info' | 'error' | 'success';

export const log = (message: string, type: LogType = 'info'): void => {
  const ts = new Date().toISOString();
  const tag = type.toUpperCase();
  console.log(`[${ts}] [${tag}] ${message}`);
};
