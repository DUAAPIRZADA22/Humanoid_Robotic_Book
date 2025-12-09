"""
Simple Chat API Backend for Railway Deployment
A Flask-based API that provides mock AI responses for the humanoid robotics chatbot
"""

from flask import Flask, request, jsonify, Response
from flask_cors import CORS
import json
import os
import time
import random
from datetime import datetime

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Mock responses for the humanoid robotics chatbot
MOCK_RESPONSES = [
    "Humanoid robotics combines artificial intelligence with mechanical engineering to create robots that can perform human-like tasks. These robots use advanced sensors, actuators, and control systems to navigate and interact with their environment.",
    "The field of humanoid robotics has made significant advances in recent years, with robots like Boston Dynamics' Atlas and Honda's ASIMO demonstrating incredible capabilities in balance, locomotion, and human interaction.",
    "AI integration in humanoid robots typically involves computer vision for perception, machine learning for decision-making, and natural language processing for human-robot interaction. These systems work together to enable autonomous behavior.",
    "Control systems for humanoid robots use techniques like inverse kinematics, balance control, and gait planning to achieve stable and efficient movement. Modern robots can walk, run, climb stairs, and even perform complex maneuvers.",
    "The future of humanoid robotics includes applications in healthcare assistance, disaster response, space exploration, and domestic help. As AI and robotics technology continue to advance, we'll see more sophisticated and capable humanoid robots in everyday life."
]

def generate_mock_response(question):
    """Generate a contextual mock response based on the question"""
    responses = MOCK_RESPONSES.copy()
    random.shuffle(responses)

    # Add some context from the user's question
    if "control" in question.lower():
        responses.insert(0, "Control systems are fundamental to humanoid robotics. They involve feedback loops, PID controllers, and advanced algorithms to maintain balance and execute precise movements. Modern control systems can adapt to changing conditions in real-time.")
    elif "ai" in question.lower() or "artificial intelligence" in question.lower():
        responses.insert(0, "AI powers the 'brain' of humanoid robots. Through machine learning and neural networks, robots can learn from experience, recognize patterns, and make decisions. Computer vision allows them to see and understand their environment.")
    elif "sensors" in question.lower():
        responses.insert(0, "Sensors are the robot's 'senses' - they include cameras for vision, microphones for hearing, LiDAR for distance measurement, accelerometers for balance, and tactile sensors for touch. These sensors provide the data needed for intelligent decision-making.")

    return responses[0]

@app.route('/api/v1/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'timestamp': datetime.utcnow().isoformat(),
        'version': '1.0.0',
        'service': 'humanoid-robotics-chat-api'
    })

@app.route('/api/v1/chat', methods=['POST'])
def chat():
    """Main chat endpoint"""
    try:
        data = request.get_json()

        if not data or 'question' not in data:
            return jsonify({'error': 'Question is required'}), 400

        question = data['question']
        stream = data.get('stream', False)

        if stream:
            # Return streaming response
            def generate():
                response_text = generate_mock_response(question)

                # Send start event
                yield f"event: start\n"
                yield f"data: {json.dumps({'type': 'start', 'sessionId': f'sess_{int(time.time())}'})}\n\n"

                # Send content in chunks
                words = response_text.split()
                current_chunk = ""

                for i, word in enumerate(words):
                    current_chunk += word + " "

                    # Send chunk every 3-5 words
                    if (i + 1) % 4 == 0 or i == len(words) - 1:
                        yield f"event: chunk\n"
                        yield f"data: {json.dumps({'type': 'chunk', 'content': current_chunk.strip()})}\n\n"
                        current_chunk = ""
                        time.sleep(0.1)  # Simulate processing time

                # Send end event
                yield f"event: end\n"
                yield f"data: {json.dumps({'type': 'end'})}\n\n"

            return Response(generate(), mimetype='text/plain')
        else:
            # Return regular JSON response
            response_text = generate_mock_response(question)

            return jsonify({
                'answer': response_text,
                'sources': [],
                'sessionId': f'sess_{int(time.time())}',
                'timestamp': datetime.utcnow().isoformat()
            })

    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/v1/models', methods=['GET'])
def get_models():
    """Get available models"""
    return jsonify({
        'models': [
            {
                'id': 'mock-humanoid-ai',
                'name': 'Mock Humanoid Robotics AI',
                'description': 'A mock AI model specializing in humanoid robotics and AI integration topics'
            }
        ]
    })

@app.route('/api/v1/status', methods=['GET'])
def get_status():
    """Get API status"""
    return jsonify({
        'status': 'operational',
        'uptime': 'active',
        'version': '1.0.0',
        'endpoints': {
            'health': '/api/v1/health',
            'chat': '/api/v1/chat',
            'models': '/api/v1/models',
            'status': '/api/v1/status'
        }
    })

@app.errorhandler(404)
def not_found(error):
    return jsonify({'error': 'Endpoint not found'}), 404

@app.errorhandler(500)
def internal_error(error):
    return jsonify({'error': 'Internal server error'}), 500

if __name__ == '__main__':
    port = int(os.environ.get('PORT', 7860))
    app.run(host='0.0.0.0', port=port, debug=False)