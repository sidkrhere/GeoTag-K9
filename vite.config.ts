import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// https://vitejs.dev/config/
export default defineConfig({
  
  base:'/GeoTag-K9/',
  server: {
    port: 5173,
    open: false
  },
  plugins: [react()]
});
