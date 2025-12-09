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
4. **Important: Set Root Directory to `backend`**
   - In project settings → Variables
   - Set `ROOT_DIRECTORY` = `backend`
   - Or during setup, specify "Root Path" = `backend`
5. **Add Environment Variables (optional):**
   - `PORT`: `7860`
   - `FLASK_ENV`: `production`
6. **Deploy**

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
1. **Check root directory**: Make sure it's set to `backend`
2. **Verify files**: Ensure `Dockerfile` exists in `backend/` directory
3. **Check logs**: Use Railway dashboard logs for error details
4. **Port issues**: Make sure app listens on `$PORT` environment variable

## 🧪 Local Testing

Test locally before deploying:
```bash
cd backend
python app.py
```
Then test: `http://localhost:7860/api/v1/health`
