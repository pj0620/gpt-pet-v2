import { fileURLToPath } from "node:url";
import tailwindcss from "@tailwindcss/vite";
import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

// https://vite.dev/config/
export default defineConfig({
  plugins: [react(), tailwindcss()],
  resolve: {
    alias: { "@": fileURLToPath(new URL("./src", import.meta.url)) },
  },
  server: {
    port: 5173,
    strictPort: true,
    proxy: {
      // The pet server (`gpt-pet serve`) runs on :8080; the simulator owns :8000.
      // SSE streams through http-proxy unbuffered; timeouts are disabled so the
      // /api/events connection is never cut by the proxy.
      "/api": { target: "http://127.0.0.1:8080", changeOrigin: true, timeout: 0, proxyTimeout: 0 },
    },
  },
});
