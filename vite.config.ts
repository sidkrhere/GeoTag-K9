import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [react()],
  base:'/GeoTag-K9/'
  server: {
    port: 5173,
    open: false
  }
});
