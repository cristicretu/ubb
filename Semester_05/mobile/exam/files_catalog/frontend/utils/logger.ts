type LogType = 'info' | 'error' | 'success';

export const log = (message: string, type: LogType = 'info'): void => {
  const timestamp = new Date().toISOString();
  const prefix = `[${timestamp}]`;
  
  switch (type) {
    case 'error':
      console.error(`${prefix} [ERROR] ${message}`);
      break;
    case 'success':
      console.log(`${prefix} [SUCCESS] ${message}`);
      break;
    case 'info':
    default:
      console.log(`${prefix} [INFO] ${message}`);
      break;
  }
};
