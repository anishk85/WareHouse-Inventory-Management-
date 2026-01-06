import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  turbopack: {},
  
  allowedDevOrigins: [
    '10.76.114.60',
    '10.146.198.60',
    '10.174.141.60',
    'localhost',
  ],
};

export default nextConfig;
