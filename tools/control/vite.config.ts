import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// Plain http dev server (default) so the browser allows ws:// to the device
// without tripping the mixed-content block that https would.
export default defineConfig({
  plugins: [react()],
});
