# Humanoid Robotics Chat API Backend

A simple Flask-based API backend for the Humanoid Robotics chatbot, designed for deployment on Railway.

## Features

- RESTful API endpoints for chat functionality
- Server-Sent Events (SSE) support for streaming responses
- CORS enabled for frontend integration
- Mock AI responses specialized in humanoid robotics topics
- Health check and status endpoints

## API Endpoints

### Health Check
```
GET /api/v1/health
```

### Chat (Streaming)
```json
POST /api/v1/chat
Content-Type: application/json

{
  "question": "How do humanoid robots maintain balance?",
  "stream": true
}
```

### Chat (Regular Response)
```json
POST /api/v1/chat
Content-Type: application/json

{
  "question": "What sensors do humanoid robots use?",
  "stream": false
}
```

### Available Models
```
GET /api/v1/models
```

### API Status
```
GET /api/v1/status
```

## Deployment on Railway

1. Push this code to your GitHub repository
2. Connect your repository to Railway
3. Set the root directory to `backend`
4. Railway will automatically detect the Python app and deploy

## Environment Variables

- `PORT`: Port number (Railway sets this automatically)
- `FLASK_ENV`: Set to `production` for production deployment

## Local Development

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Run the app:
```bash
python app.py
```

The API will be available at `http://localhost:7860`
