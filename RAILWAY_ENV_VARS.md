# Railway Environment Variables Guide

## 🚀 Required Environment Variables for Railway

### **Essential Variables (Must Add)**

| Variable | Value | Purpose |
|----------|-------|---------|
| `PORT` | `7860` | Port that Railway expects the app to listen on |
| `ROOT_DIRECTORY` | `backend` | Forces Railway to use backend directory (CRITICAL) |
| `PYTHONUNBUFFERED` | `1` | Prevents Python from buffering output |
| `PYTHONDONTWRITEBYTECODE` | `1` | Prevents Python from writing .pyc files |

### **Optional Variables**

| Variable | Value | Purpose |
|----------|-------|---------|
| `FLASK_ENV` | `production` | Sets Flask to production mode |
| `RAILWAY_ENVIRONMENT` | `production` | Custom environment indicator |

## 🔧 How to Add Environment Variables

### **Method 1: Railway Dashboard**
1. Go to your Railway project
2. Click **"Settings"** tab
3. Click **"Variables"** in the left menu
4. Click **"New Variable"**
5. Add each variable from the table above

### **Method 2: Railway CLI**
```bash
railway login
railway variables set PORT=7860
railway variables set ROOT_DIRECTORY=backend
railway variables set PYTHONUNBUFFERED=1
railway variables set PYTHONDONTWRITEBYTECODE=1
```

### **Method 3: railway.toml (Already Added)**
Some variables are already set in your `railway.toml` file:
```toml
[env]
PORT = "7860"
PYTHONUNBUFFERED = "1"
PYTHONDONTWRITEBYTECODE = "1"
```

## ⚠️ Critical Notes

### **ROOT_DIRECTORY = backend (Most Important)**
This variable forces Railway to:
- Ignore the root Dockerfile (ROS/Ubuntu)
- Use the backend directory as the project root
- Build the Flask app instead of the ROS environment

### **PORT = 7860**
Railway automatically sets this, but adding it ensures the app works correctly.

## 🧪 Testing Environment Variables

After setting variables, you can verify them:

### **In Railway Dashboard:**
1. Go to "Settings" → "Variables"
2. Check that all variables are listed
3. Deploy the project

### **In the App:**
The app should show these variables in startup logs.

## 🔍 Common Issues & Solutions

### **Issue: Variables Not Applied**
**Solution**: Delete the deployment and redeploy after adding variables

### **Issue: ROOT_DIRECTORY Not Working**
**Solution**: 
1. Set ROOT_DIRECTORY in both Variables AND Build Settings
2. Use CLI: `railway variables set ROOT_DIRECTORY=backend`

### **Issue: App Not Starting**
**Solution**: Check that all essential variables are set correctly

## ✅ Quick Setup Checklist

- [ ] `PORT` = `7860`
- [ ] `ROOT_DIRECTORY` = `backend` 
- [ ] `PYTHONUNBUFFERED` = `1`
- [ ] `PYTHONDONTWRITEBYTECODE` = `1`
- [ ] Root Directory set to `backend` in Build Settings
- [ ] Delete old deployment
- [ ] Redeploy project

Once these are set, your Railway deployment should work perfectly!
