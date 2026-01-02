# Railway Deployment Guide

## 🚀 Deploy Backend to Railway

### Option 1: Using Railway CLI (Recommended)

1. **Install Railway CLI:**
```bash
npm install -g @railway/cli
```

2. **Login to Railway:**
```bash
railway login
```

3. **Navigate to backend directory:**
```bash
cd backend
```

4. **Initialize Railway project:**
```bash
railway init
```

5. **Deploy:**
```bash
railway up
```

### Option 2: Using Railway Dashboard

1. **Go to [railway.app](https://railway.app)**
2. **Click "New Project" → "Deploy from GitHub repo"**
3. **Select your repository**
4. **⚠️ CRITICAL: Set Root Directory to `backend`**
   - In project settings → Build Settings
   - Set "Root Directory" = `backend`
   - This forces Railway to ignore the root Dockerfile
5. **Verify Build Settings:**
   - Builder: Nixpacks (recommended)
   - Start Command: `gunicorn app:app --bind 0.0.0.0:$PORT --workers 4 --timeout 120`
6. **Add Environment Variables:**
   - `PORT`: `7860`
   - `PYTHONUNBUFFERED`: `1`
   - `PYTHONDONTWRITEBYTECODE`: `1`
7. **Deploy**

## 🔧 Configuration Details

- **Root Directory**: `backend` (very important!)
- **Build Command**: `pip install -r requirements.txt`
- **Start Command**: `gunicorn app:app --bind 0.0.0.0:$PORT --workers 4 --timeout 120`
- **Port**: 7860 (Railway sets this automatically)

## 📋 What Railway Uses

Railway will use the following files in the backend directory:
- `Dockerfile` - For containerization
- `requirements.txt` - Python dependencies
- `Procfile` - Process management
- `.dockerignore` - Files to exclude

## ✅ Post-Deployment

Once deployed:
1. Get your Railway URL from the dashboard
2. Test the health endpoint: `https://your-app-name.up.railway.app/api/v1/health`
3. Update frontend config in `docusaurus.config.js`:
   ```javascript
   chatApiEndpoint: 'https://your-app-name.up.railway.app/api/v1'
   ```

## 🐛 Troubleshooting

If deployment fails:

### **ISSUE: Railway using wrong Dockerfile**
**Error**: Shows Ubuntu/ROS installation instead of Python
**Solution**:
1. Go to Railway project → Settings → Build Settings
2. Set "Root Directory" to `backend`
3. Delete the deployment and redeploy

### **ISSUE: Root directory not respected**
**Solution**:
1. In Railway dashboard → Project Settings → Variables
2. Add `ROOT_DIRECTORY` = `backend`
3. Or use Railway CLI: `railway variables set ROOT_DIRECTORY=backend`

### **Other Issues**:
1. **Check root directory**: Make sure it's set to `backend`
2. **Verify files**: Ensure `app.py` and `requirements.txt` exist in `backend/` directory
3. **Check logs**: Use Railway dashboard logs for error details
4. **Port issues**: Make sure app listens on `$PORT` environment variable
5. **Builder issues**: Switch to "Nixpacks" builder instead of Dockerfile

## 🧪 Local Testing

Test locally before deploying:
```bash
cd backend
python app.py
```
Then test: `http://localhost:7860/api/v1/health`
