/**
 * Docusaurus Translation Service
 * Main entry point
 */

import dotenv from 'dotenv';
import Fastify from 'fastify';
import cors from '@fastify/cors';
import helmet from '@fastify/helmet';
import rateLimit from '@fastify/rate-limit';
import pino from 'pino';
import { translateRoutes } from './api/translate.js';
import { initDatabase } from './db/client.js';

// Load environment variables
dotenv.config();

// Logger configuration
const logger = pino({
  level: process.env.LOG_LEVEL || 'info',
  transport: {
    target: 'pino-pretty',
    options: {
      colorize: true,
      translateTime: 'HH:MM:ss Z',
      ignore: 'pid,hostname',
    },
  },
});

// Create Fastify instance
const fastify = Fastify({
  logger,
  trustProxy: true,
});

/**
 * Start the server
 */
async function start() {
  const port = parseInt(process.env.PORT) || 3001;
  const host = process.env.HOST || '0.0.0.0';

  // Register plugins
  await fastify.register(helmet, {
    contentSecurityPolicy: false, // Disabled for easier development
  });

  await fastify.register(cors, {
    origin: (process.env.ALLOWED_ORIGINS || 'http://localhost:3000').split(','),
    credentials: true,
  });

  await fastify.register(rateLimit, {
    max: parseInt(process.env.RATE_LIMIT_MAX) || 100,
    timeWindow: parseInt(process.env.RATE_LIMIT_WINDOW) || 60000,
    errorResponseBuilder: (request, context) => ({
      success: false,
      error: 'Rate limit exceeded',
      retryAfter: context.ttl,
    }),
  });

  // Initialize database
  await initDatabase();

  // Register routes
  await fastify.register(translateRoutes, { prefix: '/api' });

  // Root endpoint
  fastify.get('/', async (request, reply) => {
    return {
      service: 'Docusaurus Translation Service',
      version: '1.0.0',
      endpoints: {
        translate: '/api/translate',
        health: '/api/health',
        cacheStats: '/api/cache/stats',
        batch: '/api/translate/batch',
      },
      docs: 'https://github.com/your-repo/docs',
    };
  });

  // Start listening
  try {
    await fastify.listen({ port, host });
    logger.info(`🚀 Translation service running on http://${host}:${port}`);
    logger.info(`📚 API documentation: http://${host}:${port}/`);
  } catch (error) {
    logger.error('Failed to start server:', error);
    process.exit(1);
  }
}

// Handle graceful shutdown
const gracefulShutdown = async (signal) => {
  logger.info(`Received ${signal}, shutting down gracefully...`);
  await fastify.close();
  process.exit(0);
};

process.on('SIGTERM', () => gracefulShutdown('SIGTERM'));
process.on('SIGINT', () => gracefulShutdown('SIGINT'));

// Start the server
start();
