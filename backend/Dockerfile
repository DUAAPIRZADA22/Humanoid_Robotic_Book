# ==============================================
# FastAPI Backend for Humanoid Robotics Book
# Railway-compatible Dockerfile
# Ubuntu 22.04 + Python 3.10
# ==============================================

FROM ubuntu:22.04

# Build argument for cache busting - pass different value to force rebuild
ARG BUILD_TIMESTAMP=2024-12-30-01:20-FORCE-REBUILD

# Add label with timestamp to force cache invalidation
LABEL build.timestamp="${BUILD_TIMESTAMP}"
LABEL build.cache_bust="true"

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install Python 3.10 and all system dependencies
# Note: Ubuntu 22.04 ships with Python 3.10 by default
# Added libpq-dev for psycopg2-binary support
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    build-essential \
    libpq-dev \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create symbolic links for python/pip (Ubuntu 22.04 uses python3)
RUN ln -sf /usr/bin/python3 /usr/bin/python && \
    ln -sf /usr/bin/pip3 /usr/bin/pip

# Verify Python version
RUN python --version && pip --version

# Set working directory
WORKDIR /app

# Cache busting - use timestamp in ENV to invalidate all subsequent layers
ENV CACHE_BUST="${BUILD_TIMESTAMP}"

# Cache busting - use timestamp in a RUN command to invalidate cache
RUN echo "Build timestamp: ${BUILD_TIMESTAMP}" && \
    echo "Cache bust: ${CACHE_BUST}"

# Copy requirements first for better caching
COPY requirements.txt .

# Upgrade pip and install Python dependencies
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Create non-root user for security
RUN useradd -m -u 1000 appuser && \
    chown -R appuser:appuser /app

# Set environment variables
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PORT=8000

# Switch to non-root user
USER appuser

# Expose the port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD curl -f http://localhost:${PORT:-8000}/health || exit 1

# Run the FastAPI application with uvicorn
CMD exec uvicorn main:app --host 0.0.0.0 --port ${PORT:-8000}
