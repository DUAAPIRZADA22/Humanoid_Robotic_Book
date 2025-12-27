# Railway Environment Variables - Complete Setup (Qdrant + Cohere)

## 🚀 **Essential Environment Variables for Railway**

### **🔥 Critical for Deployment**
| Variable | Value | Purpose |
|----------|-------|---------|
| `ROOT_DIRECTORY` | `backend` | Forces Railway to use backend folder (MOST IMPORTANT) |
| `PORT` | `7860` | Port that Railway expects |
| `PYTHONUNBUFFERED` | `1` | Prevents Python output buffering |
| `PYTHONDONTWRITEBYTECODE` | `1` | Prevents .pyc file generation |

---

## 🤖 **AI Services Environment Variables**

### **Cohere API (Required for AI)**
| Variable | Value | Purpose |
|----------|-------|---------|
| `COHERE_API_KEY` | `your_cohere_api_key_here` | Cohere AI API key for embeddings and reranking |

**How to get Cohere API Key:**
1. Go to [cohere.com](https://cohere.com)
2. Sign up and go to Dashboard → API Keys
3. Create a new API key

---

## 🗄️ **Vector Database Environment Variables**

### **Qdrant Configuration (Required for RAG)**
| Variable | Value | Purpose |
|----------|-------|---------|
| `QDRANT_HOST` | `localhost` or `your-qdrant-url.railway.app` | Qdrant server host |
| `QDRANT_PORT` | `6333` | Qdrant server port |
| `QDRANT_API_KEY` | `your_qdrant_api_key_if_cloud` | Qdrant API key (if using cloud) |

**Qdrant Setup Options:**

**Option 1: Railway Hosted Qdrant**
1. Deploy Qdrant as separate Railway project
2. Get the Railway URL: `your-qdrant-app.up.railway.app`
3. Set: `QDRANT_HOST` = `your-qdrant-app.up.railway.app`

**Option 2: Qdrant Cloud**
1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create a cluster
3. Get cluster URL and API key
4. Set: `QDRANT_HOST` = `your-cluster-url.qdrant.tech`
5. Set: `QDRANT_API_KEY` = `your_qdrant_api_key`

**Option 3: Skip for Now (Mock Mode)**
- Set: `USE_RERANK` = `false`
- Backend will work with mock responses

---

## ⚙️ **Application Configuration**

### **App Settings (Optional)**
| Variable | Value | Purpose |
|----------|-------|---------|
| `APP_NAME` | `Physical AI Chatbot` | Application name |
| `APP_VERSION` | `1.0.0` | Application version |
| `DEBUG` | `false` | Debug mode (production: false) |
| `HOST` | `0.0.0.0` | Server host |
| `FLASK_ENV` | `production` | Flask environment |

### **CORS Settings**
| Variable | Value | Purpose |
|----------|-------|---------|
| `CORS_ORIGINS` | `["https://your-vercel-app.vercel.app", "http://localhost:3000"]` | Allowed origins |

---

## 🧠 **RAG Configuration (Advanced)**

### **Chunking & Retrieval**
| Variable | Default | Purpose |
|----------|---------|---------|
| `MIN_CHUNK_SIZE` | `50` | Minimum text chunk size |
| `MAX_CHUNK_SIZE` | `1000` | Maximum text chunk size |
| `CHUNK_OVERLAP` | `100` | Overlap between chunks |
| `TOP_K_RESULTS` | `10` | Number of retrieved documents |
| `SIMILARITY_THRESHOLD` | `0.7` | Minimum similarity score |

### **Model Configuration**
| Variable | Default | Purpose |
|----------|---------|---------|
| `EMBEDDING_MODEL` | `embed-english-v3.0` | Cohere embedding model |
| `RERANK_MODEL` | `rerank-english-v3.0` | Cohere reranking model |
| `USE_RERANK` | `true` | Enable/disable reranking |

---

## 🚀 **Quick Setup Tiers**

### **🟢 Tier 1: Basic Mock (No API Keys)**
```
ROOT_DIRECTORY=backend
PORT=7860
PYTHONUNBUFFERED=1
PYTHONDONTWRITEBYTECODE=1
```

### **🟡 Tier 2: Cohere Only (AI + Simple Storage)**
```
ROOT_DIRECTORY=backend
PORT=7860
PYTHONUNBUFFERED=1
PYTHONDONTWRITEBYTECODE=1
COHERE_API_KEY=your_cohere_api_key
USE_RERANK=false
```

### **🔴 Tier 3: Full RAG (Cohere + Qdrant)**
```
ROOT_DIRECTORY=backend
PORT=7860
PYTHONUNBUFFERED=1
PYTHONDONTWRITEBYTECODE=1
COHERE_API_KEY=your_cohere_api_key
QDRANT_HOST=your-qdrant-url.railway.app
QDRANT_PORT=6333
USE_RERANK=true
```

---

## 🔧 **How to Add in Railway**

### **Railway Dashboard:**
1. Project → Settings → Variables
2. Add each variable from your chosen tier

### **Railway CLI:**
```bash
railway variables set ROOT_DIRECTORY=backend
railway variables set PORT=7860
railway variables set COHERE_API_KEY=your_key_here
# Add more variables as needed
```

---

## ⚠️ **Important Notes**

### **Security:**
- Never commit API keys to Git
- Use Railway's encrypted environment variables
- Rotate API keys regularly

### **Cost Management:**
- Cohere and Qdrant Cloud have usage-based pricing
- Monitor usage in their dashboards
- Set up alerts if needed

### **Development vs Production:**
- Development: Can use localhost Qdrant
- Production: Use cloud Qdrant or separate Railway deployment

---

## ✅ **Deployment Checklist**

**Basic Setup:**
- [ ] ROOT_DIRECTORY=backend
- [ ] PORT=7860
- [ ] PYTHONUNBUFFERED=1
- [ ] PYTHONDONTWRITEBYTECODE=1

**AI Setup (Optional):**
- [ ] COHERE_API_KEY=your_key
- [ ] QDRANT_HOST=your_qdrant_url
- [ ] USE_RERANK=true/false

**After Setup:**
- [ ] Delete old Railway deployment
- [ ] Redeploy project
- [ ] Test health endpoint: `/api/v1/health`
- [ ] Test chat functionality

Start with **Tier 1** to get deployment working, then add API keys for full AI functionality!
