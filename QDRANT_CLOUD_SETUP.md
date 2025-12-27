# Qdrant Cloud Setup Guide

This guide walks you through setting up Qdrant Cloud for production deployment of your Physical AI & Humanoid Robotics Book chatbot.

## Why Use Qdrant Cloud?

✅ **Managed Service**: No need to maintain servers
✅ **High Availability**: 99.9% uptime SLA
✅ **Scaling**: Automatic scaling based on load
✅ **Security**: Built-in authentication and encryption
✅ **Performance**: Optimized for vector operations
✅ **Backups**: Automated backups and point-in-time recovery

## Quick Start

### 1. Create a Qdrant Cloud Account

1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Sign up with your email or GitHub
3. Verify your email address

### 2. Create a Cluster

1. Click "Create Cluster"
2. Choose your configuration:
   - **Region**: Select closest to your users
   - **Plan**: Free tier includes 1GB storage (great for testing)
   - **Cluster Name**: `robotics-book-db` (or your preferred name)

3. Click "Create Cluster"
4. Wait for deployment (2-5 minutes)

### 3. Get Connection Details

Once your cluster is ready:

1. Go to your cluster dashboard
2. Click "Connect" or "API Keys"
3. Copy your **Cluster URL** (looks like: `https://your-cluster-name.cloud.qdrant.io`)
4. Generate and copy your **API Key**

### 4. Configure Your Application

Update your `.env` file:

```bash
# For Qdrant Cloud
QDRANT_URL=https://your-cluster-name.cloud.qdrant.io
QDRANT_API_KEY=your_actual_api_key_here

# Optional performance settings
QDRANT_TIMEOUT=30
QDRANT_COLLECTION_NAME=humanoid_robotics_book
QDRANT_VECTOR_SIZE=1024
```

### 5. Test the Connection

```bash
cd backend
python test_qdrant.py
```

You should see: `[OK] Connected to Qdrant at your-cluster.cloud.qdrant.io`

## Detailed Configuration

### Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `QDRANT_URL` | Yes (Cloud) | - | Your Qdrant Cloud URL |
| `QDRANT_API_KEY` | Yes (Cloud) | - | Authentication API key |
| `QDRANT_TIMEOUT` | No | 30 | Request timeout in seconds |
| `QDRANT_COLLECTION_NAME` | No | humanoid_robotics_book | Default collection name |
| `QDRANT_VECTOR_SIZE` | No | 1024 | Embedding dimension |
| `QDRANT_DISTANCE_METRIC` | No | cosine | Similarity metric |
| `QDRANT_ON_DISK` | No | true | Persist vectors to disk |

### Using with Local Development

For development, you can switch between local and cloud:

```bash
# Development (local)
QDRANT_HOST=localhost
QDRANT_PORT=6333
# QDRANT_URL= (leave empty)

# Production (cloud)
# QDRANT_URL=https://your-cluster.cloud.qdrant.io
# QDRANT_API_KEY=your_api_key
# QDRANT_HOST= (will be ignored)
```

## Deployment Options

### 1. Railway Deployment

Add these to your Railway environment variables:

```bash
QDRANT_URL=https://your-cluster-name.cloud.qdrant.io
QDRANT_API_KEY=your_api_key
COHERE_API_KEY=your_cohere_api_key
```

### 2. Vercel Deployment

In your Vercel project settings:

```bash
QDRANT_URL=https://your-cluster-name.cloud.qdrant.io
QDRANT_API_KEY=your_api_key
```

### 3. Docker Deployment

```dockerfile
# Dockerfile
ENV QDRANT_URL=https://your-cluster-name.cloud.qdrant.io
ENV QDRANT_API_KEY=${QDRANT_API_KEY}
```

```bash
docker run -e QDRANT_API_KEY=your_key your-app
```

## Performance Optimization

### 1. Choose the Right Plan

| Plan | Storage | RAM | CPU | Best For |
|------|---------|-----|-----|-----------|
| Free | 1GB | 512MB | 1 vCPU | Development & small projects |
| Standard | 10GB | 2GB | 2 vCPUs | Production apps |
| Premium | 100GB | 8GB | 4 vCPUs | Large scale applications |

### 2. Optimize Your Configuration

```python
# In your application
vector_store = QdrantVectorStore(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
    collection_name="robotics_book",
    # Optimized settings for cloud
    prefer_grpc=True,  # Use gRPC for better performance
    timeout=30,
    https=True  # Always use HTTPS for cloud
)
```

### 3. Batch Operations

```python
# Process documents in batches
batch_size = 100
for i in range(0, len(documents), batch_size):
    batch = documents[i:i+batch_size]
    await vector_store.upsert(batch)
```

## Monitoring

### 1. Qdrant Cloud Dashboard

Monitor:
- Storage usage
- Query performance
- Request count
- Error rates

### 2. Application Monitoring

Add metrics to your app:

```python
from qdrant_client.http import models

# Get collection info
info = await vector_store.get_collection_info()
print(f"Vectors stored: {info.points_count}")
print(f"Storage used: {info.points_count * 1024} bytes (approx)")
```

## Security Best Practices

### 1. API Key Management

✅ **DO**:
- Store API keys in environment variables
- Use Railway's encrypted secrets
- Rotate keys regularly
- Use different keys for dev/prod

❌ **DON'T**:
- Commit keys to Git
- Hard-code keys in code
- Share keys publicly
- Use default keys

### 2. Network Security

```python
# Always use HTTPS in production
QDRANT_URL=https://your-cluster.cloud.qdrant.io

# Never use HTTP for cloud connections
# WRONG: QDRANT_URL=http://your-cluster.cloud.qdrant.io
```

### 3. Access Control

```python
# Implement rate limiting
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=60

# Add API authentication
API_KEY_REQUIRED=true
API_KEY_HEADER=X-API-Key
```

## Troubleshooting

### Common Issues

1. **"Connection refused"**
   - Check if QDRANT_URL is correct
   - Verify API key is valid
   - Ensure cluster is running

2. **"Invalid API key"**
   - Generate a new key in dashboard
   - Check for trailing spaces
   - Verify key is not expired

3. **"Timeout errors"**
   - Increase `QDRANT_TIMEOUT`
   - Check network connectivity
   - Reduce batch sizes

4. **"Storage limit exceeded"**
   - Check usage in dashboard
   - Delete old collections
   - Upgrade your plan

### Debug Commands

```bash
# Test connection
python -c "
import os
from rag.vector_store import QdrantVectorStore
import asyncio

async def test():
    store = QdrantVectorStore(
        url=os.getenv('QDRANT_URL'),
        api_key=os.getenv('QDRANT_API_KEY')
    )
    health = await store.health_check()
    print(health)

asyncio.run(test())
"
```

## Migration from Local Qdrant

### Export Data from Local

```python
# Export script
from rag.vector_store import QdrantVectorStore

local_store = QdrantVectorStore(
    host="localhost",
    port=6333,
    collection_name="robotics_book"
)

# Retrieve all documents
all_docs = []
# Add your retrieval logic here

# Save to file
import json
with open('documents.json', 'w') as f:
    json.dump(all_docs, f)
```

### Import to Cloud

```python
# Import script
cloud_store = QdrantVectorStore(
    url="https://your-cluster.cloud.qdrant.io",
    api_key="your_api_key",
    collection_name="robotics_book"
)

# Load from file
with open('documents.json', 'r') as f:
    docs = json.load(f)

# Upload in batches
await cloud_store.upsert(docs)
```

## Cost Optimization

### 1. Monitor Usage

Track:
- Document count
- Query frequency
- Storage growth

### 2. Optimize Storage

```python
# Use smaller embeddings where possible
EMBEDDING_MODEL=embed-english-light-v3.0  # 384 dim vs 1024 dim

# Enable on-disk storage
QDRANT_ON_DISK=true

# Clean up old data regularly
```

### 3. Query Optimization

```python
# Use similarity thresholds
results = await vector_store.search(
    query_vector=embedding,
    score_threshold=0.7,  # Filter out weak matches
    top_k=5  # Limit results
)
```

## Support Resources

- [Qdrant Cloud Documentation](https://qdrant.tech/documentation/cloud/)
- [Qdrant Pricing](https://qdrant.tech/pricing/)
- [Support: support@qdrant.tech](mailto:support@qdrant.tech)
- [Discord Community](https://discord.gg/qdrant)

## Next Steps

1. ✅ Create Qdrant Cloud account
2. ✅ Set up your cluster
3. ✅ Configure environment variables
4. ✅ Test connection
5. 🔄 Deploy to production
6. 🔄 Monitor performance
7. 🔄 Optimize based on usage

Your Qdrant Cloud instance is now ready for production use with your humanoid robotics book chatbot! 🚀