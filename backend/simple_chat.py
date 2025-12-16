"""
Simple FastAPI backend for Physical AI & Humanoid Robotics book chatbot.
Provides basic chat responses without requiring full RAG pipeline setup.
"""

import os
import asyncio
import json
from typing import AsyncGenerator, Dict, Any
import logging
from dotenv import load_dotenv

# Load environment variables from .env
load_dotenv()

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import uvicorn

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    query: str
    top_k: int = 5
    similarity_threshold: float = 0.7
    use_rerank: bool = True


# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Book Chatbot - Simple",
    description="Simple chatbot backend for the Physical AI & Humanoid Robotics book",
    version="1.0.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://localhost:3001",
        "http://localhost:7860",
        "https://humanoid-robotic-book.vercel.app"
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)


def generate_intelligent_response(query: str) -> str:
    """
    Generate intelligent responses based on the user's query about Physical AI and Humanoid Robotics.
    """
    query_lower = query.lower()

    # Physical AI and Humanoid Robotics related responses
    if any(keyword in query_lower for keyword in ['what is', 'define', 'explain']):
        if 'physical ai' in query_lower:
            return """**Physical AI** refers to artificial intelligence systems that interact with and understand the physical world through sensors and actuators. Unlike purely digital AI, Physical AI is embodied in robots or other physical systems that can perceive, reason about, and act in real-world environments.

Key aspects of Physical AI include:
- **Embodiment**: AI systems with physical bodies that can interact with the world
- **Perception**: Understanding the environment through sensors like cameras, LiDAR, and tactile sensors
- **Action**: The ability to manipulate objects and move through physical space
- **Real-time Response**: Processing information and making decisions in real-time

Physical AI bridges the gap between digital intelligence and physical interaction, enabling applications like autonomous vehicles, robotic assistants, and smart manufacturing systems."""

        elif 'humanoid robot' in query_lower:
            return """**Humanoid Robots** are robots designed to resemble and mimic human body structure and behavior. These robots typically have a torso, head, two arms, and two legs, allowing them to perform human-like movements and tasks.

Key characteristics of humanoid robots include:
- **Bipedal Locomotion**: Walking on two legs using complex balance and gait control
- **Upper Body Dexterity**: Arms and hands capable of manipulation and grasping
- **Human-like Perception**: Vision systems and sensors that process the world similarly to humans
- **Social Interaction**: Designed to work alongside humans in various environments

Applications of humanoid robots include:
- Healthcare assistance and elderly care
- Manufacturing and industrial tasks
- Search and rescue operations
- Education and entertainment
- Space exploration

Famous examples include Boston Dynamics' Atlas, Honda's ASIMO, and Tesla's Optimus."""

        elif 'robotics' in query_lower:
            return """**Robotics** is the interdisciplinary field that involves the design, construction, operation, and use of robots. It combines engineering, computer science, AI, and many other disciplines to create machines that can perform tasks autonomously or with human guidance.

Major areas in robotics include:

**1. Mechanical Design**: Creating the physical structure and mechanisms
**2. Electronics and Sensors**: Hardware for perception and control
**3. Control Systems**: Algorithms that govern robot movement and behavior
**4. Artificial Intelligence**: Enabling robots to learn, adapt, and make decisions
**5. Human-Robot Interaction**: Designing interfaces and protocols for human-robot collaboration

Modern robotics spans from industrial manufacturing arms to autonomous vehicles, from medical surgical robots to home assistants, continuing to revolutionize how we work and live."""

    elif any(keyword in query_lower for keyword in ['how', 'learn', 'start', 'begin', 'course']):
        return """**Getting Started with Physical AI and Humanoid Robotics**

This course provides a comprehensive introduction to the exciting field of Physical AI and Humanoid Robotics. Here's how to make the most of your learning journey:

## 📚 **Course Structure**
- **Module 1: Foundations** - Learn the basics of robotics, AI, and control systems
- **Module 2: Perception Systems** - Computer vision, sensor fusion, and state estimation
- **Module 3: Motion Planning** - Path planning, trajectory optimization, and control
- **Module 4: Machine Learning for Robotics** - Deep learning, reinforcement learning, and adaptation
- **Module 5: Human-Robot Interaction** - Social robotics, safety, and collaboration

## 🛠️ **Prerequisites**
- Basic programming knowledge (Python recommended)
- Mathematics fundamentals (linear algebra, calculus, probability)
- Physics and mechanics understanding
- Eagerness to learn and experiment!

## 🚀 **Learning Approach**
1. **Theory First**: Understand the concepts and mathematical foundations
2. **Hands-on Practice**: Implement algorithms and work with simulations
3. **Real-world Projects**: Apply your knowledge to practical robotics challenges
4. **Stay Updated**: Robotics is rapidly evolving - keep learning!

Would you like me to elaborate on any specific module or topic?"""

    elif any(keyword in query_lower for keyword in ['sensor', 'perception', 'vision']):
        return """**Robot Perception and Sensing Systems**

Robots perceive the world through various sensors that act as their "senses." Modern humanoid robots use sophisticated sensor systems:

## 📷 **Visual Sensors**
- **RGB Cameras**: Standard color imaging for object recognition
- **Depth Cameras**: Intel RealSense, Kinect for 3D perception
- **LiDAR**: Light Detection and Ranging for precise distance measurement
- **Stereo Vision**: Paired cameras for depth perception

## 🔊 **Audio Sensors**
- **Microphones**: Speech recognition and sound localization
- **Acoustic Sensors**: Environment understanding and navigation

## 👆 **Tactile Sensors**
- **Force/Torque Sensors**: Measure interaction forces
- **Pressure Sensors**: Detect contact and pressure distribution
- **Proximity Sensors**: Detect nearby objects without contact

## 🧭 **Proprioception**
- **IMUs (Inertial Measurement Units)**: Orientation and acceleration
- **Encoders**: Joint position and velocity measurement
- **GPS**: Outdoor positioning (for mobile robots)

## 🧠 **Sensor Fusion**
Combining multiple sensor inputs creates a coherent understanding of the environment using techniques like:
- Kalman Filters
- Particle Filters
- Bayesian Networks
- Deep Learning approaches

This sensor integration enables robust perception even when individual sensors fail or provide noisy data."""

    elif any(keyword in query_lower for keyword in ['control', 'movement', 'walk', 'balance']):
        return """**Robot Control and Movement Systems**

Control systems are the "brain and nervous system" that enable robots to move with precision and grace.

## 🎯 **Control Hierarchy**

**High-level Control**: Strategic planning and decision-making
- Task planning and behavior selection
- Path planning and navigation
- Human-robot interaction logic

**Mid-level Control**: Motion planning and coordination
- Trajectory generation and optimization
- Gait planning for legged robots
- Coordination between multiple joints

**Low-level Control**: Individual joint/motor control
- PID controllers for precise position/velocity control
- Force control for safe interaction
- Real-time feedback loops (1000Hz+)

## 🚶 **Bipedal Walking Control**

Humanoid walking involves complex control challenges:
- **Balance Control**: Maintaining center of gravity within support polygon
- **Gait Generation**: Creating stable walking patterns
- **Impact Absorption**: Softening foot strikes for energy efficiency
- **Adaptation**: Adjusting to uneven terrain and disturbances

## 🧮 **Control Techniques**
- **Classical Control**: PID, LQR, MPC (Model Predictive Control)
- **Adaptive Control**: Learning and adjusting control parameters
- **Robust Control**: Handling uncertainties and disturbances
- **Learning-based Control**: Reinforcement learning, imitation learning

Modern humanoid robots like Boston Dynamics' Atlas demonstrate incredibly sophisticated control, enabling dynamic movements like running, jumping, and backflips!"""

    elif any(keyword in query_lower for keyword in ['ai', 'machine learning', 'neural', 'deep learning']):
        return """**AI and Machine Learning in Robotics**

Artificial Intelligence transforms robots from pre-programmed machines into adaptive, intelligent systems that can learn and improve.

## 🤖 **Key AI Applications in Robotics**

### **1. Computer Vision**
- Object detection and recognition
- Scene understanding and segmentation
- Visual odometry and SLAM (Simultaneous Localization and Mapping)
- Human pose estimation for interaction

### **2. Reinforcement Learning**
- Learning optimal control policies through trial and error
- Reward-based learning for complex tasks
- Simulation-to-real transfer learning
- Continuous improvement through experience

### **3. Imitation Learning**
- Learning from human demonstrations
- Behavior cloning for task reproduction
- Learning complex skills efficiently
- Safe learning through human guidance

### **4. Deep Learning**
- Neural networks for perception and control
- End-to-end learning from sensors to actions
- Feature extraction from raw sensor data
- Pattern recognition in complex environments

## 🧠 **Neural Network Architectures**
- **CNNs**: Image processing and visual perception
- **RNNs/LSTMs**: Sequential decision making and memory
- **Transformers**: Attention mechanisms for complex reasoning
- **Graph Neural Networks**: Understanding spatial relationships

## 🎯 **Challenges in Robotic AI**
- **Sample Efficiency**: Learning from limited real-world data
- **Safety and Reliability**: Ensuring consistent, predictable behavior
- **Real-time Performance**: Making decisions within time constraints
- **Generalization**: Adapting to new, unseen situations

The future of robotics lies in combining classical engineering approaches with modern AI to create truly intelligent, adaptive machines."""

    else:
        return """I'm here to help you learn about **Physical AI and Humanoid Robotics**! 🤖

This comprehensive course covers everything from basic robotics concepts to advanced AI applications in humanoid robots.

## 🔍 **Popular Topics You Can Ask About:**

**Foundations:**
- "What is Physical AI?"
- "Explain humanoid robots"
- "How do robots work?"

**Technical Areas:**
- Robot sensors and perception systems
- Movement and control algorithms
- AI and machine learning in robotics
- Human-robot interaction

**Learning:**
- "How should I start learning robotics?"
- "What are the prerequisites?"
- "What projects can I work on?"

**Advanced Topics:**
- Reinforcement learning for robots
- Computer vision in robotics
- Bipedal walking and balance
- Robotic manipulation

Feel free to ask any specific questions about these topics, or let me know what aspect of Physical AI and Humanoid Robotics interests you most! I'm here to support your learning journey. 🚀"""


@app.post("/chat")
async def chat_stream(request: ChatRequest) -> StreamingResponse:
    """
    Stream chat responses using Server-Sent Events (SSE).
    """
    async def generate_response() -> AsyncGenerator[str, None]:
        """Generate streaming response."""
        try:
            # Send initial metadata
            metadata = {
                "type": "metadata",
                "query": request.query,
                "top_k": request.top_k,
                "similarity_threshold": request.similarity_threshold,
                "use_rerank": request.use_rerank
            }
            yield f"data: {json.dumps(metadata)}\n\n"

            logger.info(f"Generating response for query: {request.query}")

            # Generate intelligent response
            response_text = generate_intelligent_response(request.query)

            # Send response as content chunk
            chunk_data = {
                "type": "content",
                "index": 0,
                "content": response_text,
                "score": 1.0,
                "source": "Physical AI Knowledge Base",
                "headers": ["Physical AI & Humanoid Robotics"]
            }
            yield f"data: {json.dumps(chunk_data)}\n\n"

            # Send completion event
            completion = {
                "type": "complete",
                "total_chunks": 1,
                "message": "Response generation complete"
            }
            yield f"data: {json.dumps(completion)}\n\n"

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            error_data = {
                "type": "error",
                "message": str(e)
            }
            yield f"data: {json.dumps(error_data)}\n\n"

    return StreamingResponse(
        generate_response(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no"
        }
    )


@app.get("/health")
async def health_check() -> Dict[str, Any]:
    """Health check endpoint."""
    return {
        "status": "healthy",
        "version": "1.0.0-simple",
        "components": {
            "chatbot": True,
            "api": True
        }
    }


@app.get("/")
async def root() -> Dict[str, str]:
    """Root endpoint."""
    return {
        "message": "Physical AI & Humanoid Robotics Book Chatbot API - Simple Mode",
        "version": "1.0.0-simple",
        "docs": "/docs",
        "endpoints": {
            "chat": "/chat",
            "health": "/health"
        }
    }


if __name__ == "__main__":
    # Run the app
    uvicorn.run(
        "simple_chat:app",
        host="0.0.0.0",
        port=7861,  # Different port to avoid conflict
        reload=False,
        access_log=True
    )